#include "draw_rects.h"
#include <string>
#include <vector>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui.hpp>

float colors_z[6][3] = { {1,0,1}, {0,0,1},{0,1,1},{0,1,0},{1,1,0},{1,0,0} };
float DrawRects::get_color(int c, int x, int max)
{
    float ratio = ((float)x/max)*5;
    int i = floor(ratio);
    int j = ceil(ratio);
    ratio -= i;
    float r = (1-ratio) * colors_z[i][c] + ratio*colors_z[j][c];
    //printf("%f\n", r);
    return r*255;
}

//const int        DrawRects::kRectangleThickness = 2;//3;
DrawRects::DrawRects()
{
    kRectangleThickness = 2;
}

DrawRects::~DrawRects()
{
}

std::string DrawRects::GetClassString(int type)
{
    switch (type)
    {
        case PERSON: return "person";
        case BICYCLE: return "bicycle";
        case MOTORBIKE: return "motorbike";
        case CAR: return "car";
        case TRUCK: return "truck";
        case BUS: return "bus";
        default:return "error";
    }
}
int DrawRects::GetClassInt(int type)
{
    switch (type)
    {
        case PERSON: return 0;
        case BICYCLE: return 1;
        case MOTORBIKE: return 3;
        case CAR: return 2;
        case TRUCK: return 7;
        case BUS: return 5;
        default:return 0;
    }
}

void DrawRects::DrawImageRect(
    const std::vector<MvObject2D> &detected_objects,
    cv::Mat &image,
    int RectangleThickness)
{
    if (detected_objects.size() == 0){
        return;
    }

    // Draw rectangles for each object
    for (int kk = 0;kk<detected_objects.size();kk++)
    {
        if (detected_objects[kk].object.x >= 0
            && detected_objects[kk].object.y >= 0
            && detected_objects[kk].object.width > 0
            && detected_objects[kk].object.height > 0)
        {
            // Draw object information label
            //DrawLabel(detected_objects[kk], image);

            //printf("<MIIVII-DEBUG> [%s] %d ImgW=%d ImgH=%d RECT(x=%d y=%d w=%d h=%d) \n", __FUNCTION__, __LINE__,image.cols, image.rows,detected_objects[kk].object.x, detected_objects[kk].object.y, detected_objects[kk].object.width, detected_objects[kk].object.height);

            int x2 = detected_objects[kk].object.x + detected_objects[kk].object.width;
            if (x2 >= image.cols)
                x2 = image.cols - 1;

            int y2 = detected_objects[kk].object.y + detected_objects[kk].object.height;
            if (y2 >= image.rows)
                y2 = image.rows - 1;

            int offset  = detected_objects[kk].type * 123457 % 80;
            float red   = get_color(2, offset, 80);
            float green = get_color(1, offset, 80);
            float blue  = get_color(0, offset, 80);

            cv::Mat image_roi = image(cv::Rect(cv::Point(detected_objects[kk].object.x, detected_objects[kk].object.y),  cv::Point(x2, y2)));
            cv::Mat color_fill(image_roi.size(), CV_8UC3,  cv::Scalar(red, green, blue));

            double alpha = 0.15;
            cv::addWeighted(color_fill, alpha, image_roi, 1.0 - alpha , 0.0, image_roi);

            // Draw rectangle
            cv::rectangle(image,
                          cv::Point(detected_objects[kk].object.x, detected_objects[kk].object.y),
                          cv::Point(x2, y2),
                          cv::Scalar(cv::Scalar(red, green, blue)),
                          RectangleThickness,
                          CV_AA,
                          0);
        }
    }
}

void DrawRects::DrawLabel(MvObject2D &in_detected_object,
                          cv::Mat &image)
{
    cv::Point rectangle_origin(in_detected_object.object.x, in_detected_object.object.y);
    // label's property
    const int font_face = cv::FONT_HERSHEY_DUPLEX;
    const double font_scale = 1.0;//0.7;
    const int font_thickness = 1;
    int font_baseline = 0;
    int icon_width = 40;
    int icon_height = 40;
    std::ostringstream label_one;
    //std::ostringstream label_two;
    cv::Size label_size = cv::getTextSize("0123456789",
                                          font_face,
                                          font_scale,
                                          font_thickness,
                                          &font_baseline);
    cv::Point label_origin = cv::Point(rectangle_origin.x,
                                       rectangle_origin.y - font_baseline - kRectangleThickness*2 - icon_height);
    label_one << GetClassString(in_detected_object.type)<<"  "<<std::to_string(in_detected_object.score);
    if (in_detected_object.objectID > 0)
    {
        label_one << " " << std::to_string(in_detected_object.objectID);
    }
    if(label_origin.x < 0)
        label_origin.x = 0;
    if(label_origin.y < 0)
        label_origin.y = 0;
    cv::Rect text_holder_rect;
    text_holder_rect.x = label_origin.x;
    text_holder_rect.y = label_origin.y;
    text_holder_rect.width = label_size.width + icon_width;
    if (text_holder_rect.x + text_holder_rect.width > image.cols)
        text_holder_rect.width = image.cols - text_holder_rect.x - 1;
    text_holder_rect.height = label_size.height + icon_height;
    if (text_holder_rect.y + text_holder_rect.height > image.rows)
        text_holder_rect.height = image.rows - text_holder_rect.y - 1;
    cv::Mat roi = image(text_holder_rect);
    cv::Mat text_holder (roi.size(), CV_8UC3, cv::Scalar(0,0,0));
    double alpha = 0.3;
    cv::addWeighted(text_holder, alpha, roi, 1.0 - alpha, 0.0, roi);
    label_origin.x+= icon_width;
    label_origin.y+= text_holder_rect.height / 3;
    cv::putText(image,
                label_one.str(),
                label_origin,
                font_face,
                font_scale,
                CV_RGB(255, 255, 255),
                1,
                CV_AA);
    label_origin.y+= text_holder_rect.height / 3;
    // cv::putText(image,
    //             label_two.str(),
    //             label_origin,
    //             font_face,
    //             font_scale,
    //             CV_RGB(255, 255, 255),
    //             1,
    //             CV_AA);
} // DrawRects::DrawLabel()
