#ifndef __DISPLAY_VISP_H__
#define __DISPLAY_VISP_H__
#include "libauto_tracker/tracking_events.h"
#include <visp/vpColor.h>
#include <visp/vpPlot.h>

namespace tracking_events{
  class DisplayVispEvents : public tracking::EventsBase{
  private:
    bool flush_display_;
    vpPlot* plot_;
    int iter_;

    void on_detect_pattern_generic(const unsigned int iter,const vpImage<vpRGBa>& I,const vpCameraParameters& cam, detectors::DetectorBase& detector, const vpColor& color);
  public:
    DisplayVispEvents(bool flush_display=true);
    ~DisplayVispEvents();
    virtual void on_finished();
    virtual void on_initial_waiting_for_pattern(const vpImage<vpRGBa>& I);

    virtual void on_detect_pattern(const unsigned int iter,const vpImage<vpRGBa>& I,const  vpCameraParameters& cam, detectors::DetectorBase& detector);
    virtual void on_redetect_pattern(const unsigned int iter,const vpImage<vpRGBa>& I,const  vpCameraParameters& cam, detectors::DetectorBase& detector, const vpRect& detection_region);
    virtual void on_detect_model(const vpImage<vpRGBa>& I,
                                 const  vpCameraParameters& cam,
                                 const vpHomogeneousMatrix& cMo,
                                 vpMbTracker& mbt,
                                 const std::vector<vpImagePoint>& model_inner_corner,
                                 const std::vector<vpImagePoint>& model_outer_corner);
    virtual void on_track_model(const unsigned int iter,const vpImage<vpRGBa>& I,const vpCameraParameters& cam, const vpHomogeneousMatrix& cMo, vpMbTracker& mbt);
  };
}
#endif
