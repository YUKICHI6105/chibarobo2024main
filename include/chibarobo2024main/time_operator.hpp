#pragma once

#ifndef TIME_OPERATOR_HPP
#define TIME_OPERATOR_HPP

#include <rclcpp/rclcpp.hpp>

// rclcpp::Timeの演算子オーバーロード
inline bool operator>(const rclcpp::Time& lhs, const rclcpp::Time& rhs) {
    return (lhs.seconds() > rhs.seconds()) || 
           (lhs.seconds() == rhs.seconds() && lhs.nanoseconds() > rhs.nanoseconds());
}

inline bool operator<(const rclcpp::Time& lhs, const rclcpp::Time& rhs) {
    return (lhs.seconds() < rhs.seconds()) || 
           (lhs.seconds() == rhs.seconds() && lhs.nanoseconds() < rhs.nanoseconds());
}

inline bool operator>=(const rclcpp::Time& lhs, const rclcpp::Time& rhs) {
    return (lhs.seconds() > rhs.seconds()) || 
           (lhs.seconds() == rhs.seconds() && lhs.nanoseconds() >= rhs.nanoseconds());
}

inline bool operator<=(const rclcpp::Time& lhs, const rclcpp::Time& rhs) {
    return (lhs.seconds() < rhs.seconds()) || 
           (lhs.seconds() == rhs.seconds() && lhs.nanoseconds() <= rhs.nanoseconds());
}

inline bool operator==(const rclcpp::Time& lhs, const rclcpp::Time& rhs) {
    return lhs.seconds() == rhs.seconds() && lhs.nanoseconds() == rhs.nanoseconds();
}

inline bool operator!=(const rclcpp::Time& lhs, const rclcpp::Time& rhs) {
    return lhs.seconds() != rhs.seconds() || lhs.nanoseconds() != rhs.nanoseconds();
}

#endif  // TIME_OPERATOR_HPP