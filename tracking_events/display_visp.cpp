#include "cv.h"

#include "display_visp.h"
#include "libauto_tracker/tracking.h"
#include <visp/vpDisplay.h>
#include <visp/vpMeterPixelConversion.h>
#include <vector>
#include <cassert>
#include <fstream>

namespace tracking_events{
  DisplayVispEvents:: DisplayVispEvents(bool flush_display) : flush_display_(flush_display),plot_(NULL),iter_(0){

  }

  DisplayVispEvents:: ~DisplayVispEvents(){
    if(plot_) delete plot_;
  }
  void DisplayVispEvents:: on_finished(){
    tracking::Tracker_::statistics_t& statistics = fsm_->get_statistics();
    std::cout << "statistics:" << std::endl;
    std::cout << "\tglobal:" << std::endl;
    std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var) << std::endl;
    std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var) << std::endl;
    std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var) << std::endl;

    std::cout << "\tX:" << std::endl;
    std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_x) << std::endl;
    std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_x) << std::endl;
    std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_x) << std::endl;

    std::cout << "\tY:" << std::endl;
    std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_y) << std::endl;
    std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_y) << std::endl;
    std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_y) << std::endl;

    std::cout << "\tZ:" << std::endl;
    std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_z) << std::endl;
    std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_z) << std::endl;
    std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_z) << std::endl;

    std::cout << "\tW_X:" << std::endl;
    std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_wx) << std::endl;
    std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_wx) << std::endl;
    std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_wx) << std::endl;

    std::cout << "\tW_Y:" << std::endl;
    std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_wy) << std::endl;
    std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_wy) << std::endl;
    std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_wy) << std::endl;

    std::cout << "\tW_Z:" << std::endl;
    std::cout << "\t\tmedian:" << boost::accumulators::median(statistics.var_wz) << std::endl;
    std::cout << "\t\tmean:" << boost::accumulators::mean(statistics.var_wz) << std::endl;
    std::cout << "\t\tmax:" << boost::accumulators::max(statistics.var_wz) << std::endl;

  }

  void DisplayVispEvents:: on_initial_waiting_for_pattern(const vpImage<vpRGBa>& I){
    vpDisplay::display(I);
    if(flush_display_) vpDisplay::flush(I);
  }

  void DisplayVispEvents:: on_detect_pattern_generic(const unsigned int iter,const vpImage<vpRGBa>& I,const  vpCameraParameters& cam, detectors::DetectorBase& detector,const vpColor& color){
    std::vector<cv::Point>& polygon = detector.get_polygon();
    if(polygon.size()==0){
      vpDisplay::displayCharString(I,vpImagePoint(0,0),"TRACKING LOST",vpColor::red);
      if(flush_display_) vpDisplay::flush(I);
      return;
    }

    const vpImagePoint corner0(polygon[0].y,polygon[0].x);
    const vpImagePoint corner1(polygon[1].y,polygon[1].x);
    const vpImagePoint corner2(polygon[2].y,polygon[2].x);
    const vpImagePoint corner3(polygon[3].y,polygon[3].x);

    std::vector<std::pair<cv::Point,cv::Point> >& lines = detector.get_lines();
    for(std::vector<std::pair<cv::Point,cv::Point> >::iterator i = lines.begin();
        i!=lines.end();
        i++
    ){
      vpDisplay::displayLine(I,vpImagePoint(i->first.y,i->first.x),vpImagePoint(i->second.y,i->second.x),color,2);
    }
    vpDisplay::displayCharString(I,corner0,"1",vpColor::blue);
    vpDisplay::displayCharString(I,corner1,"2",vpColor::yellow);
    vpDisplay::displayCharString(I,corner2,"3",vpColor::cyan);
    vpDisplay::displayCharString(I,corner3,"4",vpColor::darkRed);

    if(flush_display_) vpDisplay::flush(I);
  }
  void DisplayVispEvents:: on_detect_pattern(const unsigned int iter,const vpImage<vpRGBa>& I,const  vpCameraParameters& cam, detectors::DetectorBase& detector){
    vpDisplay::display(I);
    on_detect_pattern_generic(iter,I,cam,detector,vpColor::green);
  }

  void DisplayVispEvents:: on_redetect_pattern(const unsigned int iter,const vpImage<vpRGBa>& I,const  vpCameraParameters& cam, detectors::DetectorBase& detector, const vpRect& detection_region){
    vpDisplay::display(I);
    vpDisplay::displayRectangle(I,fsm_->get_tracking_box< vpRect > (),vpColor::orange,false,2);
    on_detect_pattern_generic(iter,I,cam,detector,vpColor::orange);
  }

  void DisplayVispEvents:: on_detect_model(const vpImage<vpRGBa>& I,
                                           const  vpCameraParameters& cam,
                                           const vpHomogeneousMatrix& cMo,
                                           vpMbTracker& mbt,
                                           const std::vector<vpImagePoint>& model_inner_corner,
                                           const std::vector<vpImagePoint>& model_outer_corner){
    vpDisplay::displayCharString(I,model_inner_corner[0],"mi1",vpColor::blue);
    vpDisplay::displayCross(I,model_inner_corner[0],2,vpColor::blue,2);
    vpDisplay::displayCharString(I,model_inner_corner[1],"mi2",vpColor::yellow);
    vpDisplay::displayCross(I,model_inner_corner[1],2,vpColor::yellow,2);
    vpDisplay::displayCharString(I,model_inner_corner[2],"mi3",vpColor::cyan);
    vpDisplay::displayCross(I,model_inner_corner[2],2,vpColor::cyan,2);
    vpDisplay::displayCharString(I,model_inner_corner[3],"mi4",vpColor::darkRed);
    vpDisplay::displayCross(I,model_inner_corner[3],2,vpColor::darkRed,2);

    vpDisplay::displayCharString(I,model_outer_corner[0],"mo1",vpColor::blue);
    vpDisplay::displayCross(I,model_outer_corner[0],2,vpColor::blue,2);
    vpDisplay::displayCharString(I,model_outer_corner[1],"mo2",vpColor::yellow);
    vpDisplay::displayCross(I,model_outer_corner[1],2,vpColor::yellow,2);
    vpDisplay::displayCharString(I,model_outer_corner[2],"mo3",vpColor::cyan);
    vpDisplay::displayCross(I,model_outer_corner[2],2,vpColor::cyan,2);
    vpDisplay::displayCharString(I,model_outer_corner[3],"mo4",vpColor::darkRed);
    vpDisplay::displayCross(I,model_outer_corner[3],2,vpColor::darkRed,2);

    mbt.display(I, cMo, cam, vpColor::blue, 1);// display the model at the computed pose.
    if(flush_display_) vpDisplay::flush(I);

  }

  void DisplayVispEvents:: on_track_model(const unsigned int iter,const vpImage<vpRGBa>& I,const  vpCameraParameters& cam, const vpHomogeneousMatrix& cMo, vpMbTracker& mbt){
    if(fsm_->get_cmd().show_plot() && plot_ == NULL){
      if(plot_) delete plot_;
      plot_ = new vpPlot(1, 700, 700, 100, 200, "Variances");
      plot_->initGraph(0,7);
    }

    vpDisplay::display(I);
    mbt.display(I, cMo, cam, vpColor::red, 1);// display the model at the computed pose.
    vpDisplay::displayFrame(I,cMo,cam,.3,vpColor::none,2);
    if(fsm_->get_cmd().using_adhoc_recovery()){
      for(unsigned int p=0;p<fsm_->get_points3D_middle().size();p++){
        vpPoint& point3D = fsm_->get_points3D_middle()[p];
        vpPoint& point3D_inner = fsm_->get_points3D_inner()[p];
        double _u=0.,_v=0.,_u_inner=0.,_v_inner=0.;

        vpMeterPixelConversion::convertPoint(cam,point3D.get_x(),point3D.get_y(),_u,_v);
        vpMeterPixelConversion::convertPoint(cam,point3D_inner.get_x(),point3D_inner.get_y(),_u_inner,_v_inner);
        int region_width= std::max((int)(std::abs(_u-_u_inner)*fsm_->get_cmd().get_adhoc_recovery_size()),1);
        int region_height=std::max((int)(std::abs(_v-_v_inner)*fsm_->get_cmd().get_adhoc_recovery_size()),1);

        int u=(int)_u;
        int v=(int)_v;
        vpDisplay::displayRectangle(
            I,
            vpImagePoint(
                std::max(v-region_height,0),
                std::max(u-region_width,0)
            ),
            vpImagePoint(
                std::min(v+region_height,(int)I.getHeight()),
                std::min(u+region_width,(int)I.getWidth())
            ),
            vpColor::cyan,
            true
            );
      }
    }
    if(flush_display_) vpDisplay::flush(I);

    vpMatrix mat = fsm_->get_mbt().getCovarianceMatrix();
    if(fsm_->get_cmd().show_plot()){
      if(fsm_->get_cmd().using_var_limit())
        plot_->plot(0,6,iter_,(double)fsm_->get_cmd().get_var_limit());
      for(int i=0;i<6;i++)
        plot_->plot(0,i,iter_,mat[i][i]);
    }


    iter_++;

  }
}
