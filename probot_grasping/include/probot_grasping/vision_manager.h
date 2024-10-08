#ifndef PROBOT_VISION_MANAGER
#define PROBOT_VISION_MANAGER

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

class VisionManager
{
  public:
	/**
   * @brief      VisionManager Constructor
   *
   * @param[in]  length   The length of the table
   * @param[in]  breadth  The breadth of the table
   */
	VisionManager(float length, float breadth);
	/**
	 * @brief      Gets the 2d location of object in camera frame
	 *
	 * @param[in]  img   The image
	 * @param      x     x postion of the object
	 * @param      y     y position of the object
	 */
	cv::Mat get2DLocation(cv::Mat img, float &x, float &y);

  private:
	/**
 	 * @brief      detect2DObject processes the image to isolate object
 	 *
 	 * @param      pixel_x  postion of the object in x-pixels
 	 * @param      pixel_y  positino of the object in y-pixels
 	 */
	cv::Mat detect2DObject(float &pixel_x, float &pixel_y, cv::Rect &tablePos);
	/**
	 * @brief      convertToMM converts pixel measurement to metric
	 *
	 * @param      pixel_mm_x  The pixel millimeters per x
	 * @param      pixel_mm_y  The pixel millimeters per y
	 */
	void convertToMM(float &pixel_mm_x, float &pixel_mm_y);
	/**
	 * @brief      detectTable isolates the table to get pixel to metric conversion
	 */
	void detectTable(cv::Rect &tablePos);
	/**
	 * @brief pixels per mm in x for the camera
	 */
	float pixels_permm_x;
	/**
	 * @brief pixels per mm in y for the camera
	 */
	float pixels_permm_y;
	/**
	 * @brief curr_pixel_centre_x is the object location in x
	 */
	float curr_pixel_centre_x;
	/**
	 * @brief curr_pixel_centre_y is the object location in y
	 */
	float curr_pixel_centre_y;
	/**
	 * @brief table length in meters
	 */
	float table_length;
	/**
	 * @brief table breadth in meters
	 */
	float table_breadth;
	/**
	 * @brief centre of the image in pixels x
	 */
	float img_centre_x_;
	/**
	 * @brief centre of the image in pixels y
	 */
	float img_centre_y_;
	/**
	 * @brief curr_img is the image currently being processed
	 */
	cv::Mat curr_img;
};

#endif