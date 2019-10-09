#ifndef DRAW_RECTS_H
#define DRAW_RECTS_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include "MvFusionUtil.h"

#define XSTR(x) #x
#define STR(x) XSTR(x)

const std::string DEFAULT_PATH  = STR(IMAGE_VIEWER_DEFAULT_PATH);

using namespace miivii;
using namespace fusion;


class DrawRects
{
public:
    DrawRects();
    ~DrawRects();
    std::string GetClassString(int type);
    int GetClassInt(int type);
    void DrawImageRect(const std::vector<MvObject2D> &detected_objects,
                            cv::Mat &image,
                            int RectangleThickness);
    void DrawLabel(MvObject2D &in_object, cv::Mat &image);
protected:
    int kRectangleThickness;//static const int kRectangleThickness;
    float get_color(int c, int x, int max);
};
#endif // DRAW_RECTS_H
