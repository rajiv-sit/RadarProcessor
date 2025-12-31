#pragma once

#include <glm/glm.hpp>

#include <cstdint>

namespace radar
{

struct RadarTrack
{
    glm::vec2 isoPosition{0.0F};  // (longitudinal, lateral) in ISO vehicle coordinates.
    glm::vec2 isoVelocity{0.0F};  // (longitudinal, lateral) in ISO vehicle coordinates.
    float length = 0.0F;
    float width = 0.0F;
    float height = 0.0F;
    float headingRad = 0.0F;      // Heading in ISO coordinates (rad), zero along +longitudinal.
    float headingRate = 0.0F;
    float probabilityOfDetection = 0.0F;
    int32_t id = -1;
    uint16_t objectClassification = 0;
    uint8_t objectClassificationConfidence = 0;
    bool isMoving = false;
    bool isStationary = false;
    bool isMoveable = false;
    bool isVehicle = false;
};

} // namespace radar
