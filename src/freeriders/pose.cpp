#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "freeriders/pose.hpp"

freeriders::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

freeriders::Pose freeriders::Pose::operator+(const freeriders::Pose& other) const {
    return freeriders::Pose(this->x + other.x, this->y + other.y, this->theta);
}

freeriders::Pose freeriders::Pose::operator-(const freeriders::Pose& other) const {
    return freeriders::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float freeriders::Pose::operator*(const freeriders::Pose& other) const { return this->x * other.x + this->y * other.y; }

freeriders::Pose freeriders::Pose::operator*(const float& other) const {
    return freeriders::Pose(this->x * other, this->y * other, this->theta);
}

freeriders::Pose freeriders::Pose::operator/(const float& other) const {
    return freeriders::Pose(this->x / other, this->y / other, this->theta);
}

freeriders::Pose freeriders::Pose::lerp(freeriders::Pose other, float t) const {
    return freeriders::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float freeriders::Pose::distance(freeriders::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float freeriders::Pose::angle(freeriders::Pose other) const { return std::atan2(other.y - this->y, other.x - this->x); }

freeriders::Pose freeriders::Pose::rotate(float angle) const {
    return freeriders::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}

std::string freeriders::format_as(const freeriders::Pose& pose) {
    // the double brackets become single brackets
    return fmt::format("freeriders::Pose {{ x: {}, y: {}, theta: {} }}", pose.x, pose.y, pose.theta);
}
