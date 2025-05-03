#include "misora2_operation/gui_tool.hpp"

Button::Button(cv::Point pos, cv::Size size) : pos(pos), size(size){}

bool Button::isClicked(const cv::Point& clickPos) const {
    return (pos.x <= clickPos.x && clickPos.x <= pos.x + size.width) && 
            (pos.y <= clickPos.y && clickPos.y <= pos.y + size.height);
}

DrawTool::DrawTool(int width, int height, cv::Scalar backgroundColor) : image(cv::Mat(height, width, CV_8UC3, backgroundColor)) {}

void DrawTool::drawImage(const cv::Mat& img, cv::Point pos, cv::Size size){
    cv::Mat resizedImg;
    cv::resize(img, resizedImg, size);
    resizedImg.copyTo(image(cv::Rect(pos,size)));
}

void DrawTool::drawText(const std::string& text, cv::Point pos, double size, cv::Scalar color, int thickness) {
    cv::putText(image, text, pos, cv::FONT_HERSHEY_SIMPLEX, size, color, thickness);
}

void DrawTool::drawAtText(const std::string& text, cv::Point pos, double size, cv::Scalar color, int thickness) {
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, size, thickness, &baseline);
    cv::Point textPos(pos.x - textSize.width / 2, pos.y + textSize.height / 2);
    cv::putText(image, text, textPos, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, size, color, thickness);
}

void DrawTool::drawButton(const Button& button, const std::string& text, cv::Scalar color, int thickness, int lineType,
                          double textSize, cv::Scalar textColor, int textThickness) {
    cv::rectangle(image, button.pos,  cv::Point(button.pos.x + button.size.width, button.pos.y + button.size.height),  color, thickness, lineType);
    cv::Point textPos(button.pos.x + button.size.width / 2, button.pos.y + button.size.height / 2);
    drawAtText(text, textPos, textSize, textColor, textThickness);
}

void DrawTool::drawButton_new(const Button& button, const std::string& text, cv::Scalar color, int thickness, int lineType,
                            double textSize, cv::Scalar textColor, int textThickness) {
    cv::rectangle(image, button.pos,  cv::Point(button.pos.x + button.size.width, button.pos.y + button.size.height),  color, thickness, lineType);
    drawText_new(text, button.pos, button.size, textSize, textColor, textThickness);
}

void DrawTool::drawText_new(const std::string& text, cv::Point pos, cv::Size size, double textSize, cv::Scalar color, int thickness){
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, textSize, thickness, &baseline);
    cv::Point tp(pos.x + (size.width - text_size.width) / 2, pos.y + (size.height + text_size.height) / 2);
    cv::putText(image, text, tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, textSize, color, thickness);
}

const cv::Mat& DrawTool::getImage() const {
    return image;
}
