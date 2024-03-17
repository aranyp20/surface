#pragma once

#include <eigen3/Eigen/Dense>



namespace common {
namespace color {


    Eigen::Vector3d hsvToRgb(const Eigen::Vector3d& hsv) {
        const auto& h = hsv[0];
        const auto& s = hsv[1];
        const auto& v = hsv[2];
        
        int h_i = int(h * 6);
        float f = h * 6 - h_i;
        float p = v * (1 - s);
        float q = v * (1 - f * s);
        float t = v * (1 - (1 - f) * s);

        switch(h_i) {
            case 0:
                return {v, t, p};
            case 1:
                return {q, v, p};
            case 2:
                return {p, v, t};
            case 3:
                return {p, q, v};
            case 4:
                return {t, p, v};
            default:
                return {v, p, q};
        }
    }


}
}