#include <cstdio>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <twitcurl.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tinyxml.h>
#include <rostweet_msgs/IncomingTweet.h>
#include <rostweet_msgs/postTweet.h>

#define TMPDIR	"/tmp"

twitCurl twitterObj;

bool postTweetCallback(rostweet_msgs::postTweet::Request  &req,
		       rostweet_msgs::postTweet::Response &res)
{
    std::string s(req.text);

    if (req.picture.size()>0) {
	    //Store pic on disk and post with picture
	    cv_bridge::CvImagePtr cv_ptr;
	    cv_ptr = cv_bridge::toCvCopy(req.picture[0]);
	    std::string picfile(std::string(TMPDIR)+"/rostweet_tmpPic.jpg");	//FIXME: use non-fixed tmp names
	    cv::imwrite(picfile.c_str(),cv_ptr->image);

	    if( twitterObj.statusUpdateWithMedia( s, picfile))
	       res.result=true;
	    else
	       res.result=false;
    } else {
	    //Standard post
	    if( twitterObj.statusUpdate( s ))
	       res.result=true;
	    else
	       res.result=false;
    }
    if (res.result) {
	std::cerr << "Tweet posted successfully" << std::endl;
    } else {
	std::cerr << "Error posting tweet:" << std::endl;
	std::string replyMsg;
	twitterObj.getLastWebResponse( replyMsg );
        std::cerr << replyMsg << std::endl;
    }
    return true;
}

/* Downloads a file from a URL using libcurl */
bool downloadMedia(std::string url, std::string filename) {
    CURL *m_curlHandle; 
    m_curlHandle = curl_easy_init(); 

    if (m_curlHandle) {
	    // Open file 
	    FILE *fp=fopen(filename.c_str(), "wb"); 
	    if( fp != NULL ) {
		    curl_easy_setopt(m_curlHandle, CURLOPT_URL, url.c_str()); 
		    curl_easy_setopt(m_curlHandle, CURLOPT_WRITEFUNCTION, NULL); 
		    curl_easy_setopt(m_curlHandle, CURLOPT_WRITEDATA, fp); 

		    // Grab image 
		    CURLcode imgresult = curl_easy_perform(m_curlHandle); 
		    fclose(fp);
		    if( !imgresult ){
			curl_easy_cleanup(m_curlHandle);
			return true;
		    } else {
			std::cerr << "downloadMedia: Cannot download the image from " << url << std::endl;
		    }
	    } else {
		std::cerr << "downloadMedia: " << filename << " file cannot be opened" << std::endl;
	    }
    }
    curl_easy_cleanup(m_curlHandle);
    return false;
}

int main( int argc, char* argv[] )
{
   ros::init(argc, argv, "rostweet");
   ros::NodeHandle nh("~");
   std::string user, password, tokenKey, tokenSecret;
   if (!nh.getParam("/rostweet/user", user)) {
	std::cout << "Enter twitter username: ";
	std::cin >> user;
   }
   if (!nh.getParam("/rostweet/password", password)) {
	std::cout << "Enter " << user << " password: ";
	char *pstr=new char[256];
	pstr=getpass("");	//TODO: getpass not portable and should be avoided
	password=std::string(pstr);
	delete pstr;
   }
   
   nh.getParam("/rostweet/tokenKey", tokenKey);
   nh.getParam("/rostweet/tokenSecret", tokenSecret);
  

    /* Set twitter username and password */
    twitterObj.setTwitterUsername( user );
    twitterObj.setTwitterPassword( password );

    /* OAuth flow begins */
    /* Step 0: Set OAuth related params. These are got by registering your app at twitter.com */
    //rostweet twitter registered app keys
    twitterObj.getOAuth().setConsumerKey( std::string( "86YcXbAyLeErF48M1NE9Q" ) );
    twitterObj.getOAuth().setConsumerSecret( std::string( "aD9P1b00gsFQ9sHsY1pCxKjYuwmcaVUxcyrhujDrg" ) );


    /* Step 1: Check if we alredy have OAuth access token */
    if( tokenKey.size() && tokenSecret.size() )
    {
        /* We already have OAuth token keys, then no need to go through auth again */
        std::cout << "Using: " << std::endl << "Key: " << tokenKey << std::endl << "Secret: " << tokenSecret << std::endl;
        twitterObj.getOAuth().setOAuthTokenKey( tokenKey );
        twitterObj.getOAuth().setOAuthTokenSecret( tokenSecret );
    }
    else
    {
	std::cout << "Getting OAuth tokens..." << std::endl;

        /* Step 2: Get request token key and secret */
	std::string tmpStr;
        twitterObj.oAuthRequestToken( tmpStr );
        twitterObj.oAuthHandlePIN( tmpStr );

        /* Step 4: Exchange request token with access token */
        twitterObj.oAuthAccessToken();

        /* Step 5: Now, save this access token key and secret for future use without PIN */
        twitterObj.getOAuth().getOAuthTokenKey( tokenKey );
        twitterObj.getOAuth().getOAuthTokenSecret( tokenSecret );

        /* Step 6: Save user, password and these keys in a param file */
        std::ofstream paramFile;
	std::string path = ros::package::getPath("rostweet") + std::string("/launch/") + user + std::string("Params.yaml");
        paramFile.open( path.c_str() );        
        paramFile.clear();
        paramFile << "user: '" << user << "'" << std::endl;
        paramFile << "tokenKey: '" << tokenKey << "'" << std::endl;
        paramFile << "tokenSecret: '" << tokenSecret << "'" << std::endl;
        paramFile.close();

	std::cout << "Your params for account " << user << " have been stored in " << path << std::endl;
	std::cout << "You can set them with rosparam if you want to skip this step in future executions" << std::endl;
    }

    std::cout << "Ready! Waiting for tweets..." << std::endl;
    /* Start tweeting */
    ros::ServiceServer service = nh.advertiseService("postTweet", postTweetCallback);  
    ros::Publisher tweetPub = nh.advertise<rostweet_msgs::IncomingTweet>("incomingTweet", 10);

    ros::Rate r(0.1);
    std::string lastTweetID("");
    while (ros::ok()) {
	    /* Get friends timeline */
	    std::string replyMsg = "";
	    if( twitterObj.timelineFriendsGet() )
	    {
		twitterObj.getLastWebResponse( replyMsg );
		//printf( "\ntwitterClient:: twitCurl::timelinePublicGet web response:\n%s\n", replyMsg.c_str() );

		TiXmlDocument xmlStream;
		const char * pReturn = xmlStream.Parse(replyMsg.c_str());
		if(pReturn != NULL) {
			TiXmlHandle hDoc(&xmlStream);
			TiXmlElement* pElem;
			pElem=hDoc.FirstChild( "statuses" ).FirstChild().Element();
			if (pElem != NULL) {
			  int nelem=0;
			  std::string firstTweetID("");
			  for( pElem; pElem; pElem=pElem->NextSiblingElement())
			  {
				TiXmlHandle hRoot(pElem);
				std::string tweetID=hRoot.FirstChild("id").Element()->GetText();
				if (!nelem) firstTweetID=tweetID;
				nelem++;
				if (tweetID==lastTweetID) break;
				else {
					rostweet_msgs::IncomingTweet itweet;
					itweet.user=hRoot.FirstChild("user").FirstChild("name").Element()->GetText();
					itweet.tweet=hRoot.FirstChild("text").Element()->GetText();

					//Get photo media. TODO: Adapt to other type of media
					TiXmlElement *mediaElem=hRoot.FirstChild("entities").FirstChild("media").FirstChild("creative").Element();
					if (mediaElem!=NULL) {
						TiXmlHandle mediaRoot(mediaElem);
						if (mediaRoot.FirstChild("type").Element()->GetText()==std::string("photo")) {
							std::string mediaurl=mediaRoot.FirstChild("media_url").Element()->GetText();
							std::string mediafile(std::string(TMPDIR)+std::string("/")+mediaurl.substr(mediaurl.find_last_of("/")+1));

							//Download
							downloadMedia(mediaurl,mediafile);

							//Convert to ros and include in tweet message
							cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
							cv_ptr->image = cv::imread(mediafile);
							itweet.picture.push_back(*(cv_ptr->toImageMsg()));
						}
					}
					tweetPub.publish(itweet);
				}
			  }
			  lastTweetID=firstTweetID;
			}
		}
	    }
	ros::spinOnce();
	r.sleep();
    } 

    return 0;
}
