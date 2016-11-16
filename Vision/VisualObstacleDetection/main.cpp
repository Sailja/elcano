/*
 * test_slic.cpp.
 *
 * Written by: Shailja.
 *
 * This file creates an over-segmentation of a provided image based on the SLIC
 * superpixel algorithm, as implemented in slic.h and slic.cpp and apply the algorithm to detect the obstacle free areas.
 */
 
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <float.h>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>

using namespace std;

#include "slic.h"

int main(int argc, char *argv[]) {
    /* Load the image and convert to Lab colour space. */
    IplImage *image = cvLoadImage(argv[1], 1);
    IplImage *lab_image = cvCloneImage(image);
    cvCvtColor(image, lab_image, CV_BGR2Lab);
    
    /* Yield the number of superpixels and weight-factors from the user. */
    int w = image->width, h = image->height;
    int nr_superpixels = atoi(argv[2]);
    int nc = atoi(argv[3]);

    double step = sqrt((w * h) / (double) nr_superpixels);
    int safe_zone_sp = 0; //total number of superpixel in safe zone
    // the following variables are used to calculate the mean of parameter values of superpixel in safe zone
    double L_m = 0, a_m = 0, b_m = 0;
    int w_m = 0, h_m = 0, d_m = 0, area_m = 0;
    /* Perform the SLIC superpixel algorithm. */
    Slic slic;
    slic.generate_superpixels(lab_image, step, nc);
    slic.create_connectivity(lab_image);
    
    /* Display the contours and show the result. */
    slic.display_contours(image, CV_RGB(255,0,0));
    //added by shailja
        //Safe zone is a trapezium 
    CvPoint pts[4] = {cv::Point( 270, 420 ), cv::Point( 370, 420 ), cv::Point( 440, 480 ), cv::Point( 200, 480 ) };
    cvFillConvexPoly(image, pts , 4, cv::Scalar(255, 255, 255),8,0);
    //L,a,b,x,y of superpixels
    double super_pixel[slic.centers.size()][7];
    // cout<<"Total number of centers are: ";
    // cout<<slic.centers.size()<<endl;
    // cout<<"L,a,b,x,y values of each superpixel centers are:"<<endl;
    for (int i = 0; i < slic.centers.size(); i++)
    {        

        // cout<<"Superpixelvalue ";
        // cout << slic.clusters[(int)slic.centers[i][3]][(int)slic.centers[i][4]];
        // cout<< slic.center_counts[i]<<" ";
        
        int width = 0;
        int center_x = (int)slic.centers[i][3];       
        while (center_x<640 && slic.clusters[center_x][(int)slic.centers[i][4]] == slic.clusters[(int)slic.centers[i][3]][(int)slic.centers[i][4]])
        {
            center_x ++;
        }
        width = (center_x - (int)slic.centers[i][3])*2;
        // cout<<width<<" ";
        int height = 0;
        int center_y = (int)slic.centers[i][4];       
        while (center_y<480 && slic.clusters[(int)slic.centers[i][3]][center_y]== slic.clusters[(int)slic.centers[i][3]][(int)slic.centers[i][4]])
        {
            center_y ++;
        }
        height = (center_y - (int)slic.centers[i][4])*2;
        // cout<<height<<" ";
        int diagonal = 0;
        int center_dy = (int)slic.centers[i][4]; 
        int center_dx = (int)slic.centers[i][3];   
        while (center_dy<480 && center_dx<640 && slic.clusters[center_dx][center_dy]== slic.clusters[(int)slic.centers[i][3]][(int)slic.centers[i][4]])
        {
            center_dy ++;
            center_dx ++;
        }
        diagonal = sqrt(pow(center_dx - (int)slic.centers[i][3], 2) + pow(center_dy
            - (int)slic.centers[i][4], 2))*2;
        // cout<<diagonal<<" ";
        // cout<<endl;
        // double v[7]={slic.centers[i][0], slic.centers[i][1], slic.centers[i][2], slic.center_counts[i], width, height, diagonal};
        super_pixel[i][0] = slic.centers[i][0];
        super_pixel[i][1] = slic.centers[i][1];
        super_pixel[i][2] = slic.centers[i][2];
        super_pixel[i][3] = slic.center_counts[i];
        super_pixel[i][4] = width;
        super_pixel[i][5] = height;
        super_pixel[i][6] = diagonal;
        CvScalar colour = cvGet2D(image, (int)slic.centers[i][4], (int)slic.centers[i][3]);//(y,x)
        if (colour.val[0] == 255 && colour.val[1] == 255 && colour.val[2] == 255)
        {
            safe_zone_sp ++;
            L_m += slic.centers[i][0];
            a_m += slic.centers[i][1];
            b_m += slic.centers[i][2];
            area_m += slic.center_counts[i];
            w_m += width;
            h_m += height;
            d_m += diagonal;            
        }
        //cout<<safe_zone_sp<<endl;
        //super_pixel.push_back(v);
        // calculating mean of (l,a,b,width,height,diag,area) of the superpixels in safe zone
        //CvScalar c1 = cvGet2D(image, 0, 0);
        // uchar blue = intensity.val[0];
        // uchar green = intensity.val[1];
        // uchar red = intensity.val[2];
        //cout<< blue<<endl;//image[((int)slic.centers[i][3])][((int)slic.centers[i][4])] << "color"<<endl;
        // if(image[(int)slic.centers[i][3]][(int)slic.centers[i][4]] == (255, 255, 255))
        // {

        // }    

    }


    //MyLine( image, cv::Point( 0, 320 ), cv::Point( 10, 10 ) );
    
    //cout<<"Superpixel value of each cluster is"<<endl; 

    // for (int i = 0; i < slic.clusters.size(); i++)
    // {
    //     for (int j = 0; j < slic.clusters[i].size(); j++)
    //     {
    //         cout << slic.clusters[i][Softwarej]<<" ";
    //     }
    //     cout<<endl;
    // }
	    
    L_m /= safe_zone_sp;
    a_m /= safe_zone_sp;
    b_m /= safe_zone_sp;
    area_m /= safe_zone_sp;
    w_m /= safe_zone_sp;
    h_m /= safe_zone_sp;
    d_m /= safe_zone_sp;
    cout<<"mean values of the parameters of superpixels in safe zone (L, a, b, area, width, height, diagonal)"<<endl;
    cout<<L_m<<" "<<a_m<<" "<<b_m<<" "<<area_m<<" "<<w_m<<" "<<h_m<<" "<<d_m<<endl;
    double ssd[slic.centers.size()];
    for (int i = 0; i < slic.centers.size(); i++)
    {
        // cout<<"Superpixel value is "<<slic.clusters[(int)slic.centers[i][3]][(int)slic.centers[i][4]];
        // cout<<endl;
        ssd[i] = pow((super_pixel[i][0]-L_m)/L_m,2) + pow((super_pixel[i][1]-a_m)/a_m,2) + pow((super_pixel[i][2]-b_m)/b_m,2)+
                 pow((super_pixel[i][3]-area_m)/area_m,2) + pow((super_pixel[i][4]-w_m)/w_m,2) + pow((super_pixel[i][5]-h_m)/h_m,2) + pow((super_pixel[i][6]-d_m)/d_m,2);
        //cout<<ssd[i]<<endl;
        if (ssd[i]<0.12) // this threshold value is hard coded 
        {
            cvCircle(image, cvPoint(slic.centers[i][3], slic.centers[i][4]), 2, (0,0,255), 2);
        }

    }    
    cvShowImage("result", image);
    cvWaitKey(0);
    cvSaveImage(argv[4], image);
}
