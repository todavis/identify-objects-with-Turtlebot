#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <string>

#include <imagePipeline.h>

//#include "opencv2/core.hpp"
//#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;

using std::cout;
using std::endl;

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"


float polygonArea(std::vector<Point2f> points)
{
    // Calculate area formed by sequence of points, 
    float area = 0.0;
 
    // Calculate value of shoelace formula
    int j = points.size() - 1;
    for (int i = 0; i < points.size(); i++)
    {
        area += (points[j].x + points[i].x) * (points[j].y - points[i].y);
        j = i;  // j is previous vertex to i
    }
 
    // Return absolute value
    return std::abs(area / 2.0);
}

float get_pt_distance(Point2f a, Point2f b)
{
    return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
}

bool isConvex(std::vector<Point2f> points)
{
	// returns if shape forms convex hull and simple angle estimate if no lines intersect
	std::vector<Point2f> hull;
	convexHull(points, hull, false);

	if (hull.size() == points.size())
	{

		float angle_sum = 0;
		float a;
		float b;
		float c;

		for (int i = 0; i < points.size(); i++)
    	{
    		int prev = i - 1;
    		int next = i + 1;

    		if (prev < 0)
    		{
    			prev = 3;
    		}

    		if (next > 3)
    		{
    			next = 0;
    		}

    		a = get_pt_distance(points[next], points[prev]);
    		b = get_pt_distance(points[i], points[prev]);
    		c = get_pt_distance(points[i], points[next]);

    		angle_sum += std::acos((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * b * c));
    	}

    	//std::cout << "angle sum: " << angle_sum << "\n";;

    	if (std::abs(angle_sum - 2 * M_PI) < .3) //close to 360 degrees
    	{
    		return true; 
    	}
    	else
    	{
    		return false;
    	}
	}
	else
	{
		return false;
	}
}

Point2f getCentroid(std::vector<Point2f> points)
{
	// gets the average points in vector
	Point2f centroid = Point2f(0, 0);
	int n = 0;

	for (int i = 0; i < points.size(); i++)
	{
		centroid.x += points[i].x;
		centroid.y += points[i].y;
		n += 1;
	}

	centroid.x /= n;
	centroid.y /= n;

	return centroid;
}

ImagePipeline::ImagePipeline(ros::NodeHandle& n)
{
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try 
    {
        if(isValid)
        {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    }
    catch (cv_bridge::Exception& e) 
    {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes)
{
    Mat scene_img = img.clone();    // fix the image

    int template_id = -1;
    if(!isValid)
    {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    }
    else if(scene_img.empty() || scene_img.rows <= 0 || scene_img.cols <= 0)
    {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << scene_img.empty() << std::endl;
        std::cout << "img.rows:" << scene_img.rows << std::endl;
        std::cout << "img.cols:" << scene_img.cols << std::endl;
    }
    else 
    {
        /***YOUR CODE HERE***/
        // Use: boxes.templates

        std::cout << "Running getTemplateID\n";

        // blur scene image
        GaussianBlur(scene_img, scene_img, Size(3, 3), 0);

        // keypoint detection for current scene
        int minHessian = 400;
        Ptr<SURF> detector = SURF::create(minHessian);

        std::vector<KeyPoint> keypoints_scene;
        Mat descriptors_scene;
        detector->detectAndCompute( scene_img, noArray(), keypoints_scene, descriptors_scene);

		//-- Draw keypoints
        //Mat img_keypoints;
        //drawKeypoints( scene_img, keypoints_scene, img_keypoints );
        //-- Show detected (drawn) keypoints
        //imshow("SURF Keypoints", img_keypoints );
        //cv::waitKey(30);

		if (keypoints_scene.size() < 100)
        {
            std::cout << "Blank Image" << "\n\n";
            return 0;
        }

        // print number of features in scene
        //std::cout << "Size of scene features:" << keypoints_scene.size() << "\n";

        float min_mse = INFINITY;
        float min_score = INFINITY;
        float area_best = 0;
        float distance_best = INFINITY;

        int img_id = 0;
        // loop through each template for feature matching
        for (int img_obj_i = 0; img_obj_i < boxes.templates.size(); img_obj_i++)
        {   
            img_id = img_obj_i + 1;
            //std::cout << "Comparing template:" << img_id << "\n";

            // keypoint detection for current object
            Mat img_object = boxes.templates[img_obj_i].clone();
            cv::resize(img_object, img_object, cv::Size(500,400)); // Resize tag image to roughly match aspect ratio on boxes
            GaussianBlur(img_object, img_object, Size(3, 3), 0);

            //std::string obj_img_name = "object_image" + std::to_string(img_id) + ".jpg";
            //imwrite(obj_img_name, img_object);

            std::vector<KeyPoint> keypoints_object;
            Mat descriptors_object;
        	detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object);

            //std::cout << "Size of obj features:" << keypoints_object.size() << "\n";

        	// feature matching and finding good matches
        	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        	std::vector <std::vector<DMatch> > knn_matches;

        	matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2);
            
            // print the number of feature matches
            //std::cout << "Size of knn_matches:" << knn_matches.size() << "\n";

            // quality of matches
        	const float ratio_thresh = 0.7f;
        	std::vector<DMatch> good_matches;

        	for (size_t i = 0; i < knn_matches.size(); i++)
            {
        		if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                {
        			good_matches.push_back(knn_matches[i][0]);
        		}
        	}

            //std::cout << "Size of good_matches:" << good_matches.size() << "\n";

        	// outlier rejection and drawing matches
            //Mat img_matches;
            //drawMatches( img_object, keypoints_object, scene_img, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
            //     Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        	// getting corresponding matching keypoints
            std::vector<Point2f> obj;
            std::vector<Point2f> scene;

            for ( size_t i = 0; i < good_matches.size(); i++ )
            {
                //-- Get the keypoints from the good matches
                obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
            }

        	// Object/Scene transformation
            Mat H; 
            Mat mask;
            double ransacReprojThreshold=3;

            if (obj.size() >= 8) 
            {
                H = findHomography( obj, scene, RANSAC, ransacReprojThreshold, mask);
            }
            else 
            {
                // Keypoint number fewer than 4, scene image unlikely to be the object image
				//std::cout << "Not enough good matches!" << "\n\n";
                continue;
            }

            // generate bounding box
            //if (img_obj_i == 13) {
            std::vector<Point2f> obj_corners(4);
            obj_corners[0] = Point2f(0, 0);
            obj_corners[1] = Point2f( (float)img_object.cols, 0 );
            obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
            obj_corners[3] = Point2f( 0, (float)img_object.rows );
            std::vector<Point2f> scene_corners(4);
            try
            {
                perspectiveTransform( obj_corners, scene_corners, H );
            }
            catch(const std::exception&)
            {   
                std::cout << "perspective transform failed!" << "\n\n";
                continue;
            }

            //check bounding box properties
            float area = polygonArea(scene_corners);
            //std::cout << "Area:" << area << "\n";

            bool is_convex = isConvex(scene_corners);
            //std::cout << "Convex:" << is_convex << "\n";

			/*if (img_id == 12)
			//{
				//-- Draw lines between the corners (the mapped object in the scene - image_2 ) for plotting
				//line( img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
				//		scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4 );
				//line( img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
				//		scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				//line( img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
				//		scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				//line( img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
				//		scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
				//-- Show detected matches

				//imshow("Good Matches & Object detection", img_matches );
				//cv::waitKey(30);
			}*/

            if (is_convex && area > 100000 )
            {
	            // Get the predicted scene key points using homography transformation from object key points

	            std::vector<Point2f> obj_inlier;
	            std::vector<Point2f> scene_inlier;
	            //std::cout << obj.size() << "\n";

	            for ( size_t i = 0; i < obj.size(); i++ )
	            {
	                if((unsigned int)mask.at<uchar>(i)) 
	                {
	                    obj_inlier.push_back(obj.at(i));
	                    scene_inlier.push_back(scene.at(i));
	                }
	            }

	            //std::cout << "size of inlier matches: " << obj_inlier.size() << "\n";

	            // draw inlier matches
	            //std::vector<DMatch> inlier_matches;

	            //for (size_t i = 0; i < good_matches.size(); i++)
	            //{
	            //    if(std::find(obj_inlier.begin(), obj_inlier.end(), keypoints_object[ good_matches[i].queryIdx ].pt) != obj_inlier.end())
	            //    {
	            //        inlier_matches.push_back(good_matches[i]);
	            //    }
	            //}

	            if (obj_inlier.size() > 6)  // <=4 means the H matrix was form by exact pt so mse=0
	            {
	                std::vector<Point2f> pred_scene(obj_inlier.size());
	                try
	                {
	                    perspectiveTransform( obj_inlier, pred_scene, H );
	                }
	                catch(const std::exception&)
	                {   
	                    std::cout << "perspective transform failed!" << "\n\n";
	                    continue;
	                }
	                
	                /*for (int pt_i=0; pt_i < obj_inlier.size(); pt_i++)
	                {
	                    drawMarker( img_object, obj_inlier[pt_i], Scalar::all(-1));
	                }
	                imshow("object frame", img_object );
	                cv::waitKey(30);
	                imwrite("object_frame.jpg", img_object);*/
	                /*
	                for (int pt_i=0; pt_i < pred_scene.size(); pt_i++)
	                {
	                    drawMarker( scene_img, pred_scene[pt_i], Scalar::all(-1), MARKER_SQUARE, 10);
	                    drawMarker( scene_img, scene_inlier[pt_i], Scalar::all(-1), MARKER_STAR, 5);
	                    line( scene_img, pred_scene[pt_i], scene_inlier[pt_i], Scalar::all(-1), 1);
	                }             
	                imshow("pred scene frame", scene_img );
	                cv::waitKey(30);
	                imwrite("pred_scene.jpg", scene_img);
	                */
	                // Evaluation for image similarity by computing mean squared distances between predicted and true scene keypoints
	                float mse = 0;
	                float curr_dist = 0;
	                int pt_num = 0;
	                float score;

	                for (int pt_i = 0; pt_i < pred_scene.size(); pt_i++)
	                {   
	                    curr_dist = get_pt_distance(pred_scene[pt_i], scene_inlier[pt_i]);
	                    if (curr_dist > 0)  // get rid of the points that are used to form the H matrix (so their errors are 0)
	                    {
	                        mse += curr_dist;
	                        pt_num += 1;
	                    }
	                }
	                mse /= pt_num;

	               	Point2f centroid = getCentroid(scene_corners);
	               	float distance_to_ctr = get_pt_distance(Point2f( (float) scene_img.cols/2, (float)scene_img.rows/2), centroid);

					//std::cout << "Distance to centre: " << distance_to_ctr << "\n";

	               	score = mse + distance_to_ctr/(scene_img.cols/4);

	                //std::cout << "score:" << score << "\n\n";

	                if (score < min_score)
	                {
	                    // update best match id
	                    template_id = img_id;
	                    min_score = score;
	                    area_best = area;
	                    distance_best = distance_to_ctr;
	                }
            	}
				else
				{
					//std::cout << "Not enough inliers!" << "\n\n";
				}
            }
			else
			{
				//std::cout << "Not convex!" << "\n\n";
			}
        }
		//std::cout << "scene_img.cols/1.2: " << scene_img.cols/1.2 << "\n";
        // check if the match box is too far (area is small) and if match center is out of scene
        // should only trigger when blank image is directly in front
	    if ( area_best < 45000 || distance_best > scene_img.cols/1.7 )
	    {
	    	std::cout << "Blank image" << "\n\n";
	    	return 0;
	    }
    }  

    std::cout << "Best match ID:" << template_id << "\n\n";
    return template_id;
}
