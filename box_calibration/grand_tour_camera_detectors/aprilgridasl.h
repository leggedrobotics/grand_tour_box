//
// Created by fu on 14/09/2024.
//

#ifndef GRAND_TOUR_CAMERA_DETECTORS_APRILGRIDASL_H
#define GRAND_TOUR_CAMERA_DETECTORS_APRILGRIDASL_H


#include <memory>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

// April tags detector and various tag families
#include "apriltags/TagDetector.h"
#include "apriltags/Tag36h11.h"
#include "calibrationtargetasl.h"

namespace cameras {

    class GridCalibrationTargetAprilgrid : public GridCalibrationTargetBase {
    public:

        typedef std::shared_ptr<GridCalibrationTargetAprilgrid> Ptr;
        typedef std::shared_ptr<const GridCalibrationTargetAprilgrid> ConstPtr;

        //target extraction options
        struct AprilgridOptions {
            AprilgridOptions() :
                    doSubpixRefinement(true),
                    maxSubpixDisplacement2(1.5),
                    showExtractionVideo(false),
                    minTagsForValidObs(4),
                    minBorderDistance(4.0),
                    blackTagBorder(2) {};

            //options
            /// \brief subpixel refinement of extracted corners
            bool doSubpixRefinement;

            /// \brief max. displacement squarred in subpixel refinement  [px^2]
            double maxSubpixDisplacement2;

            /// \brief show video during extraction
            bool showExtractionVideo;

            /// \brief min. number of tags for a valid observation
            unsigned int minTagsForValidObs;

            /// \brief min. distance form image border for valid points [px]
            double minBorderDistance;

            /// \brief size of black border around the tag code bits (in pixels)
            unsigned int blackTagBorder;

        };

        /// \brief initialize based on checkerboard geometry
        GridCalibrationTargetAprilgrid(size_t tagRows, size_t tagCols, double tagSize,
                                       double tagSpacing, const AprilgridOptions &options = AprilgridOptions());

        virtual ~GridCalibrationTargetAprilgrid() {};

        /// \brief extract the calibration target points from an image and write to an observation
        bool computeObservation(const cv::Mat &image,
                                Eigen::MatrixXd &outImagePoints,
                                std::vector<bool> &outCornerObserved) const;

    private:
        /// \brief initialize the object
        void initialize();

        /// \brief initialize the grid with the points
        void createGridPoints();

        /// \brief size of a tag [m]
        double _tagSize;

        /// \brief space between tags (tagSpacing [m] = tagSize * tagSpacing)
        double _tagSpacing;

        /// \brief target extraction options
        AprilgridOptions _options;

        // create a detector instance
        AprilTags::TagCodes _tagCodes;
        std::shared_ptr<AprilTags::TagDetector> _tagDetector;

    };
}  // namespace cameras

#endif //GRAND_TOUR_CAMERA_DETECTORS_APRILGRIDASL_H
