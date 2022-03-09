#include "MainHeader.h"

namespace imageProcessing
{
        cv::Mat imageToMatrix(const unsigned char* image, int width, int height){
        cv::Mat img = cv::Mat(cv::Size(width, height), CV_8UC4);
        img.data = (uchar *)image;
        return img;
    }

    bool detectColor(cv::Mat src, int width, int height, int color, float colorThreshold){
        cv::Mat img;
        cv::Mat img_hsv = cv::Mat(cv::Size(width, height), CV_8UC3);
        cv::cvtColor(src, img, cv::COLOR_BGR2BGRA);
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        cv::Mat filtered_img;
        int* color_bound = colors[color];
        cv::inRange(img_hsv, cv::Scalar(color_bound[0], color_bound[1], color_bound[2]), cv::Scalar(color_bound[3], color_bound[4], color_bound[5]), filtered_img);

        // std::cout << "Non zer0 pixels: " << cv::countNonZero(filtered_img) << std::endl;
        float percentage = (float)cv::countNonZero(filtered_img)/(src.cols*src.rows);
        // std::cout << "Percentage: " << percentage << std::endl;

        if (percentage > colorThreshold){
            return true;
        }
        else{
            return false;
        }
    }


    std::vector<cv::Point2f> centroidDetection(cv::Mat threshold){

        cv::Mat sobel_transform;
        vector<vector< cv::Point> > contours;
        vector<cv::Vec4i> hierachy; 

        cv::Canny( threshold, sobel_transform, 50, 150, 3);
        cv::findContours( sobel_transform, contours, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        vector <cv::Moments> mu;
        cv::Moments tempMoment;
        // XV_print(contours.size());
        for(long long unsigned int i = 0; i<contours.size(); i++){
            tempMoment =  cv::moments(contours[i], false);
            // cout<< " m00 :"<< tempMoment.m00 << endl;
            // XV_print(tempMoment.m00);
            // XV_print(tempMoment.m10);
            // XV_print(tempMoment.m01);

            if (tempMoment.m00 != 0)
            {
                // cout<< " m00 inside "<< tempMoment.m00 << "  " << typeid(tempMoment.m00).name() << endl;

                // XV_print(tempMoment.m10);
                // XV_print(tempMoment.m01);
                
                mu.push_back(tempMoment);
            }
            //debugging code
            //std::cout << "Moments" << std::endl;
            //std::cout << mu[i].m10 << " " << mu[i].m01 << " " << mu[i].m00 << " " <<std::endl;
        }

        vector<cv::Point2f> mc;
        cv::Point2f tempPoint;
        for(long long unsigned int i = 0; i<mu.size(); i++)
        {
            // XV_print(mu[i].m00);
            if (0.00001 > mu[i].m00 || mu[i].m00 > 5000000 )
            {
                // cout << "AAAA" <<endl;
                break;
            }

            tempPoint = cv::Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
            mc.push_back(tempPoint);
        }

        return mc;
    }


    cv::Point2f objectTracking(cv::Mat img, int width, int height, int object_type, webots::Display* display, bool activateDisplay){

        cv::Mat img_hsv = cv::Mat(cv::Size(width, height), CV_8UC3);
        cv::cvtColor(img, img, cv::COLOR_BGR2BGRA);
        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

        cv::Mat color_mask;

        if ( object_type == OBJECT){
            cv::inRange(img_hsv, cv::Scalar(10, 191, 0), cv::Scalar(25, 255, 255), color_mask);

        }
        else if ( (object_type == HOLE_CIRCLE) || (object_type == HOLE_SQUARE) ){
            cv::inRange(img_hsv, cv::Scalar(90, 0, 0), cv::Scalar(125, 5, 255), color_mask);
        }

        else if ( object_type == RED_BALL){
            cv::Mat color_mask1;
            //change this to filter
            cv::inRange(img_hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), color_mask);
            cv::inRange(img_hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), color_mask1);
            cv::bitwise_or(color_mask, color_mask1, color_mask);
        }
        else if ( object_type == BLUE_BALL){
            //change this to filter
            cv::inRange(img_hsv, cv::Scalar(110, 180, 150), cv::Scalar(130, 255, 255), color_mask);
        }

        std::vector<cv::Point2f> mc;
        //std::cout << mc << std::endl;
        mc =  centroidDetection(color_mask);

        //print coordinates
        // std::cout << mc << std::endl;

        //debugging code
        // std::cout <<"This is the detected Contour List :" << mc << std::endl;

        if (activateDisplay){
            cv::Mat display_image;
            cv::cvtColor(color_mask, display_image, cv::COLOR_GRAY2BGRA);
            displayImage(width, height, display_image, display);
        }

        // XV_print(mc.size())

        if(mc.size() > 0) {
            if (object_type == OBJECT){
                if (mc.size() > 1){
                    if (mc[0].x < mc[1].x){
                        return mc[0];
                    }
                    else {
                        return mc[1];
                    }
                }
                else {
                    return mc[0];
                }
            }

            else if (object_type == HOLE_CIRCLE){
                return mc[0];
            }
            else if (object_type == HOLE_SQUARE) {
                return mc[0];
            }

            else if (object_type == RED_BALL || object_type == BLUE_BALL){
                return mc[0];
            }

        }

        return cv::Point2f(-1,-1); // No object detected

    }


    void displayImage( int width, int height, cv::Mat src, webots::Display* display){
        if(src.data){
            webots::ImageRef *ir = display->imageNew(width, height, src.data, webots::Display::BGRA);
            display->imagePaste(ir, 0, 0, false);
            display->imageDelete(ir);
        }
    }


    int detectShape(cv::Mat src, int width, int height, int threshold, webots::Display* display, bool activate_display) 
    {

        cv::Mat img_hsv;
        cv::cvtColor(src, img_hsv, cv::COLOR_BGR2HSV);

        cv::Mat color_mask;
        cv::inRange(img_hsv, cv::Scalar(10, 191, 0), cv::Scalar(25, 255, 255), color_mask);

        if(activate_display){
            cv::Mat display_image;
            cv::cvtColor(color_mask, display_image, cv::COLOR_GRAY2BGRA);
            displayImage(width, height, display_image, display);       
        }

        cv::Mat sobel_transform;
        vector<vector< cv::Point> > contours;
        vector<cv::Vec4i> hierachy;

        cv::Canny( color_mask, sobel_transform, 50, 150, 3);
        cv::findContours( sobel_transform, contours, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));   

        std::vector<cv::Point2f> approx;
        std::vector<cv::Point2f> mc;
        mc =  centroidDetection(color_mask);

        
        int index;

        if (contours.size() == 1 || contours.size() == 2)
        {
            if(contours.size() == 1) 
            {
                index = 0;
            }

            else
            {
                if (mc[0].x >= mc[1].x)
                {
                    index = 0;
                }

                else
                {
                    index = 1;
                }
            }


            cv::approxPolyDP(contours[index], approx, cv::arcLength(contours[index], true)*0.01, true );
            // std::cout << "Approximation " << std::endl;
            int numLines = approx.size();
            // std::cout << numLines <<std::endl;
            
            if ( numLines > threshold ){
                return CYLINDER;
            }
            else if ( numLines <= threshold ) 
            {
                return CUBE;    
            }
            
        }

        return NO_SHAPE_DETECTED;

    }

    vector<vector<double>> detectShapes(cv::Mat src, int width, int height, int threshold, webots::Display* display, bool activate_display){
        cv::Mat img_hsv;
        cv::cvtColor(src, img_hsv, cv::COLOR_BGR2HSV);

        cv::Mat color_mask;
        cv::inRange(img_hsv, cv::Scalar(10, 191, 0), cv::Scalar(25, 255, 255), color_mask);

        if(activate_display){
            cv::Mat display_image;
            cv::cvtColor(color_mask, display_image, cv::COLOR_GRAY2BGRA);
            displayImage(width, height, display_image, display);       
        }

        cv::Mat sobel_transform;
        vector<vector< cv::Point> > contours;
        vector<cv::Vec4i> hierachy;

        cv::Canny( color_mask, sobel_transform, 50, 150, 3);
        cv::findContours( sobel_transform, contours, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));  

        vector <cv::Moments> mu;
        cv::Moments tempMoment;
        std::vector<cv::Point2f> approx;
        std::vector<int> numLines;

        for(int i = 0; i<contours.size(); i++){

            tempMoment =  cv::moments(contours[i], false);
            if (tempMoment.m00 != 0.0){
                mu.push_back(tempMoment);
                
            }
            else{
                continue;
            }
            cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[0], true)*0.01, true );
            numLines.push_back(approx.size());
            //debugging code
            //std::cout << "Moments" << std::endl;
            //std::cout << mu[i].m10 << " " << mu[i].m01 << " " << mu[i].m00 << " " <<std::endl;
        }

        std::vector< std::vector<double>> mc;
        cv::Point2f tempPoint;
        std::vector < double > tempData;
        for(int i = 0; i<mu.size(); i++){
            tempPoint = cv::Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
            tempData.push_back(tempPoint.x);
            tempData.push_back(tempPoint.y);
            // XV_print(numLines[i])
            // XV_print(tempPoint.y)
            // XV_print(tempPoint.x)



            if (numLines[i] > threshold){
                tempData.push_back((double)CYLINDER);
            }
            else{
                tempData.push_back((double)CUBE);
            }

            mc.push_back(tempData);
            tempData.clear();
        }

        return mc;        
    }


    cv::Point2f objectTrackingLeft(cv::Mat img, int width, int height, int object_type, webots::Display* display, bool activateDisplay)
    {

            cv::Mat img_hsv = cv::Mat(cv::Size(width, height), CV_8UC3);
            cv::cvtColor(img, img, cv::COLOR_BGR2BGRA);
            cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

            cv::Mat color_mask;

            if ( object_type == OBJECT){
                cv::inRange(img_hsv, cv::Scalar(10, 191, 0), cv::Scalar(25, 255, 255), color_mask);

            }
            else if ( (object_type == HOLE_CIRCLE) || (object_type == HOLE_SQUARE) ){
                cv::inRange(img_hsv, cv::Scalar(90, 0, 0), cv::Scalar(125, 5, 255), color_mask);
            }

            else if ( object_type == RED_BALL){
                cv::Mat color_mask1;
                //change this to filter
                cv::inRange(img_hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), color_mask);
                cv::inRange(img_hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), color_mask1);
                cv::bitwise_or(color_mask, color_mask1, color_mask);
            }
            else if ( object_type == BLUE_BALL){
                //change this to filter
                cv::inRange(img_hsv, cv::Scalar(110, 180, 150), cv::Scalar(130, 255, 255), color_mask);
            }

            std::vector<cv::Point2f> mc;
            //std::cout << mc << std::endl;
            mc =  centroidDetection(color_mask);

            //print coordinates
            // std::cout << mc << std::endl;

            //debugging code
            // std::cout <<"This is the detected Contour List :" << mc << std::endl;

            if (activateDisplay){
                cv::Mat display_image;
                cv::cvtColor(color_mask, display_image, cv::COLOR_GRAY2BGRA);
                displayImage(width, height, display_image, display);
            }

            // XV_print(mc.size())

            if(mc.size() > 0) {
                if (object_type == OBJECT){
                    if (mc.size() > 1){
                        if (mc[0].x > mc[1].x){
                            return mc[0];
                        }
                        else {
                            return mc[1];
                        }
                    }
                    else {
                        return mc[0];
                    }
                }

                else if (object_type == HOLE_CIRCLE){
                    return mc[0];
                }
                else if (object_type == HOLE_SQUARE) {
                    return mc[0];
                }

                else if (object_type == RED_BALL || object_type == BLUE_BALL){
                    return mc[0];
                }

            }

            return cv::Point2f(-1,-1); // No object detected

    }
}
