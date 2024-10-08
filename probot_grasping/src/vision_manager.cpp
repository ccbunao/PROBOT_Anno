#include "probot_grasping/vision_manager.h"

// 构造函数，初始化VisionManager对象
VisionManager::VisionManager(float length, float breadth)
{
	this->table_length = length;
	this->table_breadth = breadth;
}

// 获取2D位置
cv::Mat VisionManager::get2DLocation(cv::Mat img, float &x, float &y)
{
	this->curr_img = img;
	img_centre_x_ = img.rows / 2;
	img_centre_y_ = img.cols / 2;

	cv::Rect tablePos;

	// 检测桌子
	detectTable(tablePos);

	// 检测2D对象
	cv::Mat image = detect2DObject(x, y, tablePos);
	// 转换为毫米
	convertToMM(x, y);

	return image;
}

// 检测桌子
void VisionManager::detectTable(cv::Rect &tablePos)
{
	// 从图像中提取桌子并赋值给像素/毫米字段
	cv::Mat BGR[3];
	cv::Mat image = curr_img.clone();
	split(image, BGR);
	cv::Mat gray_image_red = BGR[2];
	cv::Mat gray_image_green = BGR[1];
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_red, denoiseImage, 3);

	// 阈值化图像
	cv::Mat binaryImage = denoiseImage;
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++)
		{
			int editValue = binaryImage.at<uchar>(i, j);
			int editValue2 = gray_image_green.at<uchar>(i, j);

			if ((editValue >= 0) && (editValue < 20) && (editValue2 >= 0) && (editValue2 < 20))
			{ // 检查值是否在范围内。
				binaryImage.at<uchar>(i, j) = 255;
			}
			else
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
		}
	}
	dilate(binaryImage, binaryImage, cv::Mat());

	// 获取blob的中心点
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints);
	cv::Rect bbox = cv::boundingRect(nonZeroPoints);
	cv::Point pt;
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;
	cv::circle(image, pt, 2, cv::Scalar(0, 0, 255), -1, 8);

	// 更新像素/毫米字段
	pixels_permm_y = bbox.height / table_length;
	pixels_permm_x = bbox.width  / table_breadth;

    tablePos = bbox;

	// 测试转换值
	std::cout << "Pixels in y" << pixels_permm_y << std::endl;
	std::cout << "Pixels in x" << pixels_permm_x << std::endl;

	// 绘制轮廓 - 用于调试
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(image, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
	}

	// cv::namedWindow("Table Detection", cv::WINDOW_AUTOSIZE);
	// cv::imshow("Table Detection", image);
	// cv::waitKey(100);
}

// 检测2D对象
cv::Mat VisionManager::detect2DObject(float &pixel_x, float &pixel_y, cv::Rect &tablePos)
{
	// 实现颜色阈值化和轮廓查找，以获取要抓取的2D对象的位置
	cv::Mat image, gray_image_green;
	cv::Mat BGR[3];
	image = curr_img.clone();
	cv::split(image, BGR);

	gray_image_green = BGR[1]; // 提取绿色通道，因为识别绿色物体

	// 去噪图像
	cv::Mat denoiseImage;
	cv::medianBlur(gray_image_green, denoiseImage, 3);

	// 阈值化图像
	cv::Mat binaryImage = denoiseImage;
	for (int i = 0; i < binaryImage.rows; i++)
	{
		for (int j = 0; j < binaryImage.cols; j++)
		{
			if((j<tablePos.x+3) || j>(tablePos.x+tablePos.width-3) || (i<tablePos.y+3) || i>(tablePos.y + tablePos.height-3))
			{
				binaryImage.at<uchar>(i, j) = 0;
			}
			else
			{
				int editValue = binaryImage.at<uchar>(i, j);

				if ((editValue > 100) && (editValue <= 255))
				{ // 检查值是否在范围内。
					binaryImage.at<uchar>(i, j) = 255;
				}
				else
				{
					binaryImage.at<uchar>(i, j) = 0;
				}
			}
		}
	}
	dilate(binaryImage, binaryImage, cv::Mat());

	// 获取blob的中心点
	std::vector<cv::Point> nonZeroPoints;
	cv::findNonZero(binaryImage, nonZeroPoints);
	cv::Rect bbox = cv::boundingRect(nonZeroPoints);
	cv::Point pt;
	pixel_x = bbox.x + bbox.width / 2;
	pixel_y = bbox.y + bbox.height / 2;

	// 测试转换值
	std::cout << "pixel_x" << pixel_x << std::endl;
	std::cout << "pixel_y" << pixel_y << std::endl;

	// 用于绘制
	pt.x = bbox.x + bbox.width / 2;
	pt.y = bbox.y + bbox.height / 2;
	cv::circle(image, pt, 2, cv::Scalar(0, 0, 255), -1, 8);

	// 绘制轮廓
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binaryImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(image, contours, i, color, 1, 8, hierarchy, 0, cv::Point());
	}

	//cv::namedWindow("Centre point", cv::WINDOW_AUTOSIZE);
	//cv::imshow("Centre point", image);
	//cv::waitKey(100);

	return image;
}

// 转换为毫米
void VisionManager::convertToMM(float &x, float &y)
{
	// 从像素到相机帧中的世界坐标的转换
	x = (x - img_centre_x_) / pixels_permm_x;
	y = (y - img_centre_y_) / pixels_permm_y;
}

// 临时主函数用于测试-稍后应该移除
// int main(int argc, char** argv ) {
// 	if ( argc != 2 )
//     {
//         printf("usage: VisionManager <Image_Path>\n");
//         return -1;
//     }

//     cv::Mat image;
//     image = cv::imread( argv[1], 1 );

//     if ( !image.data )
//     {
//         printf("No image data \n");
//         return -1;
//     }

//     float length = 0.3;
//     float breadth = 0.3;
//     float obj_x, obj_y;

//     VisionManager vm(length, breadth);
//     vm.get2DLocation(image, obj_x, obj_y);
//     std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
//     std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

//     cv::waitKey(0);
// }
