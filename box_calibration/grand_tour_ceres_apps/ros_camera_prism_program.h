#pragma once

#include "ros_camera_prism_parser.h"
#include "gtboxcalibration/ceresprograms.h"
#include "camera_prism_viz_interface.h"

struct ROSCameraPrismProgram : public CameraPrismProgram {
    explicit ROSCameraPrismProgram(const ROSCameraPrismParser& parser,
                                   std::unique_ptr<CameraPrismVizInterface> viz = nullptr);
    bool Solve() override;
    void visualize();
    std::map<std::string, CameraParameterPack> camera_packs;
private:
    std::unique_ptr<CameraPrismVizInterface> viz_;
};
