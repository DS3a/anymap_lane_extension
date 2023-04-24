#ifndef LANE_EXTENSION_H_
#define LANE_EXTENSION_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>



std::vector<std::vector<cv::Point>> find_contours(cv::Mat frame) {

  // std::cout << "finding contours\n";
  cv::Mat grayscale_image;

  cv::cvtColor(frame, grayscale_image, cv::COLOR_BGR2GRAY);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(grayscale_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  // std::cout << "found contours\n";
  return contours;
}

namespace lane_extension {

    std::vector<cv::Point2f> order_points_new(const std::vector<cv::Point2f>& pts) {
        // Sort the points based on their x-coordinates
        std::vector<cv::Point2f> xSorted = pts;
        std::sort(xSorted.begin(), xSorted.end(), [](cv::Point2f a, cv::Point2f b) {
            return a.x < b.x;
        });

        // Grab the left-most and right-most points from the sorted
        // x-coordinate points
        std::vector<cv::Point2f> leftMost = {xSorted[0], xSorted[1]};
        std::vector<cv::Point2f> rightMost = {xSorted[2], xSorted[3]};

        // Now, sort the left-most coordinates according to their
        // y-coordinates so we can grab the top-left and bottom-left
        // points, respectively
        std::sort(leftMost.begin(), leftMost.end(), [](cv::Point2f a, cv::Point2f b) {
            return a.y < b.y;
        });
        cv::Point2f tl = leftMost[0];
        cv::Point2f bl = leftMost[1];

        // Now, sort the right-most coordinates according to their
        // y-coordinates so we can grab the top-right and bottom-right
        // points, respectively
        std::sort(rightMost.begin(), rightMost.end(), [](cv::Point2f a, cv::Point2f b) {
            return a.y < b.y;
        });
        cv::Point2f tr = rightMost[0];
        cv::Point2f br = rightMost[1];

        // Return the coordinates in top-left, top-right,
        // bottom-right, and bottom-left order
        return {tl, tr, br, bl};
    }


    std::vector<cv::Point2f> order_points_x(std::vector<cv::Point2f> pts) {
        // sort the points based on their x-coordinates
        std::sort(pts.begin(), pts.end(), [](cv::Point2f a, cv::Point2f b) { return a.x < b.x; });

        // grab the left-most and right-most points from the sorted
        // x-roodinate points
        std::vector<cv::Point2f> leftMost(pts.begin(), pts.begin() + 2);
        std::vector<cv::Point2f> rightMost(pts.begin() + 2, pts.end());

        // now, sort the left-most coordinates according to their
        // y-coordinates so we can grab the top-left and bottom-left
        // points, respectively
        std::sort(leftMost.begin(), leftMost.end(), [](cv::Point2f a, cv::Point2f b) { return a.y < b.y; });
        cv::Point2f tl = leftMost[0];
        cv::Point2f bl = leftMost[1];

        // now, sort the right-most coordinates according to their
        // y-coordinates so we can grab the top-right and bottom-right
        // points, respectively
        std::sort(rightMost.begin(), rightMost.end(), [](cv::Point2f a, cv::Point2f b) { return a.y < b.y; });
        cv::Point2f tr = rightMost[0];
        cv::Point2f br = rightMost[1];

        // return the coordinates in top-left, top-right,
        // bottom-right, and bottom-left order
        std::vector<cv::Point2f> result{ tl, tr, br, bl };
        return result;
    }

    std::vector<cv::Mat> generate_sliding_masks(int image_size, int window_length) {

        int num_masks = image_size/window_length;
        std::vector<cv::Mat> masks(num_masks);

        for (int i=0; i<num_masks; i++) {
            masks[i] = cv::Mat::zeros(320, 320, CV_8UC1);
            masks[i].rowRange(i*window_length, (i+1)*window_length) = cv::Scalar(255);
        }

        return masks;
    }

/*
    void full_line(cv::Mat *img, cv::Point a, cv::Point b, cv::Scalar color){
        double slope = Slope(a.x, a.y, b.x, b.y);

        cv::Point p(0,0), q(img->cols,img->rows);

        p.y = -(a.x - p.x) * slope + a.y;
        q.y = -(b.x - q.x) * slope + b.y;

        line(*img,p,q,color,1,8,0);
    }
*/
    void full_line(cv::Mat* img, cv::Point a, cv::Point b, cv::Scalar color)
    {
        //points of line segment
        cv::Point p1 = a;
        cv::Point p2 = b;

        //points of line segment which extend the segment P1-P2 to
        //the image borders.
        cv::Point p,q;

        //test if line is vertical, otherwise computes line equation
        //y = ax + b
        if (p2.x == p1.x)
        {
            p = cv::Point(p1.x, 0);
            q = cv::Point(p1.x, img->rows);
        }
        else
        {
            double a = (double)(p2.y - p1.y) / (double) (p2.x - p1.x);
            double b =  p1.y - a*p1.x;

            p = cv::Point(0, b);
            q = cv::Point(img->rows, a*(img->rows) + b);

            //clipline to the image borders. It prevents a known bug on OpenCV
            //versions 2.4.X when drawing
            cv::clipLine(cv::Size(img->rows, img->cols), p, q);
        }

        // std::cout << abs(int(a.y - b.y)) << " =  thickness\n";
        cv::line(*img, p, q, color, 24);
        // cv::line(*img, p, q, color, 2);
    }

    double slope(cv::Point a, cv::Point b){
        return (double)(a.y-b.y)/(a.x-b.x);
    }


    std::vector<cv::Mat> extend_lanes(cv::Mat image, int num_masks, std::vector<cv::Mat> masks) {
        std::vector<cv::Mat> results;
        for (int i = 0; i < num_masks; i++) {
            cv::Mat result;
            cv::bitwise_and(image, image, result, masks[i]);

            auto contours = find_contours(result);
            if (contours.size() > 0) {
                // std::cout << "num contours is greater than 0\n";
                for (int j=0; j<contours.size(); j++) {
                    // auto blackbox = cv::minAreaRect(contours[i]);
                    // blackbox.points(vertices);

                    // std::cout << "finding the bounding rect\n";

                    cv::RotatedRect minRect = cv::minAreaRect(contours[j]);
                    std::vector<cv::Point2f> vertices(4);
                    minRect.points(vertices.data());

                    // std::cout << "points without sorting : " << vertices << "\n";
                    vertices = lane_extension::order_points_new(vertices);
                    // std::cout << "points with sorting : " << vertices << "\n";

                    cv::Point mid_point_a = 0.5*(vertices[0] + vertices[1]);
                    cv::Point mid_point_b = 0.5*(vertices[2] + vertices[3]);
                    int length1 = abs(vertices[0].y - vertices[3].y);
                    int breadth1 = abs(vertices[0].x - vertices[1].x);
                    int length2 = abs(vertices[1].y - vertices[2].y);
                    int breadth2 = abs(vertices[3].x - vertices[2].x);
                    int length = 0.5*(length1 + length2);
                    int breadth = 0.5*(breadth1 + breadth2);
                    std::cout << "The length and breadth are " << length << " " << breadth << std::endl;

                    float angle;

                    if (length >= 30 && breadth >= 30) {
                        if (breadth <= 130) {
                            angle = abs(atan(lane_extension::slope(mid_point_a, mid_point_b)))*180/3.14159;
                        } else if (breadth >= 130) {
                            angle = abs(90 - abs(atan(lane_extension::slope(mid_point_a, mid_point_b)))*180/3.14159);
                        } else {
                            angle = -1;
                        }
                    } else {
                        angle = -1;
                    }

                    std::cout << "the angle is : " << angle << "\n";
                    if ((angle>=75)) {
                        lane_extension::full_line(&result,
                                                  0.5*(vertices[0]+vertices[1]),
                                                  0.5*(vertices[3]+vertices[2]),
                                                  cv::Scalar(255, 255, 255));
                    } else if (angle<=15 && angle >=0) {
                        lane_extension::full_line(&result,
                                                  0.5*(vertices[0]+vertices[3]),
                                                  0.5*(vertices[2]+vertices[1]),
                                                  cv::Scalar(255, 255, 255));
                    }
                }
            }
            results.push_back(result);
            imshow("Mask " + std::to_string(i), result);
        }

        // Wait for a key press
        cv::waitKey(0);
        return results;
    }

}

#endif // LANE_EXTENSION_H_
