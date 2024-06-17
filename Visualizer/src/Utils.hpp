#pragma once

#include <imgui.h>

#include <iostream>
#include <cstdlib>
#include <cmath>

ImVec4 RGBtoHSV(const ImVec4& RGBColor)
{
    float R = RGBColor.x;
    float G = RGBColor.y;
    float B = RGBColor.z;

    float H, S, V;

    float cMax = std::max(std::max(R, G), B);
    float cMin = std::min(std::min(R, G), B);
    float delta = cMax - cMin;

    if (delta > 0)
    {
        if (cMax == R)
        {
            H = 60 * (float)(std::fmod(((G - B) / delta), 6));
        }
        else if (cMax == G)
        {
           H = 60 * (((B - R) / delta) + 2); 
        }
        else if (cMax == B)
        {
            H = 60 * (((R - G) / delta) + 4);
        }

        if (cMax > 0)
            S = delta / cMax;
        else
            S = 0;

        V = cMax;
    }
    else
    {
        H = 0;
        S = 0;
        V = cMax;
    }

    if (H < 0)
        H += 360;

    return ImVec4(H / 2.0f, S * 255.0f, V * 255.0f, 255.0f);
}
