#pragma once


namespace IMS {
    namespace FMY {

        class CvSift {
        public:
            static CvSift *getInstance();

            cv::Mat project(cv::Mat &, cv::Mat &, bool, cv::Mat &, int **, int *);

            void drawMatches(const cv::Mat &img1, const cv::Mat &img2, const cv::Mat &mask,
                             const int *pairs, const int &size, cv::Mat &output) {
                //
                int w1 = img1.cols;
                int h1 = img1.rows;
                int w2 = img2.cols;
                int h2 = img2.rows;
                int h = h1 > h2 ? h1 : h2;
                int w = w1 + w2;
                output = cv::Mat::zeros(h, w, CV_8UC3);
                cv::Rect left_roi(0, 0, w1, h1);
                cv::Mat left = output(left_roi);
                cv::Rect right_roi(w1, 0, w2, h2);
                cv::Mat right = output(right_roi);
                cv::cvtColor(img1, left, CV_GRAY2BGR);
                cv::cvtColor(img2, right, CV_GRAY2BGR);
                for (int i = 0; i < size; i += 4) {
                    int x0 = pairs[i];
                    int y0 = pairs[i + 1];
                    int x1 = pairs[i + 2];
                    int y1 = pairs[i + 3];
                    cv::circle(left, cv::Point(x0, y0), 3, cv::Scalar(255, 0, 0));
                    cv::circle(right, cv::Point(x1, y1), 3, cv::Scalar(0, 255, 0));
                    if (mask.at<char>(i / 4, 0)) {
                        cv::line(output, cv::Point(x0, y0), cv::Point(x1 + w1, y1),
                                 cv::Scalar(0, 0, 255));
                    }
                }
            }
        };
    }
}
