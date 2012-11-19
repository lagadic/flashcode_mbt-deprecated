#include "tracking_events.h"
//#include "tracking.h"

namespace tracking{

  Tracker_& EventsBase::get_tracker(){
    return *fsm_;
  }



}
