#ifndef __TRACKING_EVENTS_H__
#define __TRACKING_EVENTS_H__
#include "detectors/detector_base.h"

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpRect.h>
#include <visp/vpImagePoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMbTracker.h>
#include <vector>

namespace tracking{
  class Tracker_;
  class EventsBase{
  protected:
    tracking::Tracker_* fsm_;
  public:
    tracking::Tracker_& get_tracker();
    virtual void on_finished() = 0;
    virtual void on_initial_waiting_for_pattern(const vpImage<vpRGBa>& I) = 0;
    virtual void on_detect_pattern(const unsigned int iter,const vpImage<vpRGBa>& I, const vpCameraParameters& cam, detectors::DetectorBase& detector) = 0;
    virtual void on_redetect_pattern(const unsigned int iter,const vpImage<vpRGBa>& I,const  vpCameraParameters& cam, detectors::DetectorBase& detector, const vpRect& detection_region) = 0;
    virtual void on_detect_model(const vpImage<vpRGBa>& I,
                                 const vpCameraParameters& cam,
                                 const vpHomogeneousMatrix& cMo,
                                 vpMbTracker& mbt,
                                 const std::vector<vpImagePoint>& model_inner_corner,
                                 const std::vector<vpImagePoint>& model_outer_corner) = 0;
    virtual void on_track_model(const unsigned int iter,const vpImage<vpRGBa>& I,const vpCameraParameters& cam, const vpHomogeneousMatrix& cMo, vpMbTracker& mbt) = 0;

  friend class Tracker_;
  };
}
#endif
