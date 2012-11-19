#ifndef __EVENTS_H__
#define __EVENTS_H__
#include "cv.h"
// back-end
#include <boost/msm/back/state_machine.hpp>
//front-end
#include <boost/msm/front/state_machine_def.hpp>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMeterPixelConversion.h>
#include <boost/thread.hpp>
#include "events.h"
//#include "nodelets/controller.h"


namespace msm = boost::msm;

namespace tracking{
  struct WaitingForInput : public msm::front::state<>{
      template <class Event, class Fsm>
      void on_entry(Event const&, Fsm& ){
        std::cout <<"entering: WaitingForInput" << std::endl;
      }
      template <class Event, class Fsm>
      void on_exit(Event const& evt, Fsm& fsm){
        std::cout <<"leaving: WaitingForInput" << std::endl;
        fsm.get_tracking_events()->on_initial_waiting_for_pattern(evt.I);
      }
  };

  struct Finished : public msm::front::state<>{
    template <class Event, class Fsm>
    void on_entry(Event const& evt, Fsm& fsm){
      fsm.get_tracking_events()->on_finished();
    }
  };

  struct DetectFlashcode: public msm::front::state<> {
    template <class Fsm>
    void on_entry(finished const& evt, Fsm& fsm){}

    template <class Fsm>
    void on_exit(finished const& evt, Fsm& fsm){}

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm&)
    {
      std::cout <<"entering: DetectFlashcode" << std::endl;

    }
    template <class Event, class Fsm>
    void on_exit(Event const& evt, Fsm& fsm)
    {
      //std::cout <<"bsm="<< (unsigned long)fsm.get_tracking_events()->get_bsm() << std::endl;
      fsm.get_tracking_events()->on_detect_pattern(evt.frame,evt.I,evt.cam_,fsm.get_detector());
      std::cout <<"leaving: DetectFlashcode" << std::endl;
    }
  };
  struct ReDetectFlashcode: public msm::front::state<> {
    template <class Fsm>
    void on_entry(finished const& evt, Fsm& fsm){}

    template <class Fsm>
    void on_exit(finished const& evt, Fsm& fsm){}

    template <class Fsm>
    void on_entry(input_ready const&, Fsm&)
    {
      std::cout <<"entering: DetectFlashcode" << std::endl;

    }
    template <class Fsm>
    void on_exit(input_ready const& evt, Fsm& fsm)
    {
      //std::cout << "tracker_eventst.t:" << (unsigned long)(dynamic_cast<visp_auto_tracker::AutoTrackerNodelet&>(fsm.get_tracking_events()).tracker_) << std::endl;
      fsm.get_tracking_events()->on_redetect_pattern(evt.frame,evt.I,evt.cam_,fsm.get_detector(),fsm.template get_tracking_box< vpRect > ());
      std::cout <<"leaving: ReDetectFlashcode" << std::endl;
    }

  };

  struct DetectModel : public msm::front::state<>
  {
      template <class Fsm>
      void on_entry(finished const& evt, Fsm& fsm){}

      template <class Fsm>
      void on_exit(finished const& evt, Fsm& fsm){}

      template <class Event, class Fsm>
      void on_entry(Event const&, Fsm& fsm)
      {
        std::cout <<"entering: DetectModel" << std::endl;
      }
      template <class Event, class Fsm>
      void on_exit(Event const& evt, Fsm& fsm)
      {
        std::cout <<"leaving: DetectModel" << std::endl;
        std::vector<vpPoint>& points3D_inner = fsm.get_points3D_inner();
        std::vector<vpPoint>& points3D_outer = fsm.get_points3D_outer();

        std::vector<vpImagePoint> model_inner_corner(4);
        std::vector<vpImagePoint> model_outer_corner(4);
        for(int i=0;i<4;i++){
          vpMeterPixelConversion::convertPoint(fsm.get_cam(),points3D_outer[i].get_x(),points3D_outer[i].get_y(),model_outer_corner[i]);
          vpMeterPixelConversion::convertPoint(fsm.get_cam(),points3D_inner[i].get_x(),points3D_inner[i].get_y(),model_inner_corner[i]);
        }


        vpHomogeneousMatrix cMo;
        fsm.get_mbt().getPose(cMo);


        fsm.get_tracking_events()->on_detect_model(fsm.get_I(),fsm.get_cam(), cMo, fsm.get_mbt(), model_inner_corner,model_outer_corner);

      }
  };

  class TrackModel : public msm::front::state<>
  {
  public:
    template <class Fsm>
    void on_entry(finished const& evt, Fsm& fsm){}

    template <class Fsm>
    void on_exit(finished const& evt, Fsm& fsm){}

    template <class Event, class Fsm>
    void on_entry(Event const& evt, Fsm& fsm)
    { }
    template <class Event, class Fsm>
    void on_exit(Event const& evt, Fsm& fsm)
    {
      vpHomogeneousMatrix cMo;
      fsm.get_mbt().getPose(cMo);
      fsm.get_tracking_events()->on_track_model(evt.frame,evt.I,evt.cam_, cMo, fsm.get_mbt());
    }
  };
}
#endif /* EVENTS_H_ */
