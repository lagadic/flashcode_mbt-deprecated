#ifndef __TEVENTS_H__
#define __TEVENTS_H__
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>

namespace tracking{

  struct input_ready{
    input_ready(const vpImage<vpRGBa>& I,const vpCameraParameters& cam) : I(I),cam_(cam),frame(0) {}
    input_ready(const vpImage<vpRGBa>& I,const vpCameraParameters& cam,unsigned int frame) : I(I),cam_(cam),frame(frame) {}
    input_ready(const vpImage<vpRGBa>& I,const vpCameraParameters& cam, unsigned int frame, vpHomogeneousMatrix& cMo): I(I),cam_(cam),frame(frame), cMo(cMo) {}
    const vpImage<vpRGBa> I;
    const vpCameraParameters cam_;
    unsigned int frame;
    vpHomogeneousMatrix cMo;
  };

  struct select_input{
    select_input(vpImage<vpRGBa>& I) : I(I){}
    vpImage<vpRGBa>& I;
  };
  struct finished{
  };
}
#endif
