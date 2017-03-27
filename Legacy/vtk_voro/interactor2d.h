/*
 * File name: interactor2d.hpp
 * Date:      2012-11-22 18:31:20 +0100
 * Author:    Miroslav Kulich
 */

#ifndef _INTERACTOR2D_HPP_
#define _INTERACTOR2D_HPP_

#include <vtkInteractorStyleTrackballCamera.h>

namespace imr {

class CInteractor2d : public vtkInteractorStyleTrackballCamera {
  public:
    static CInteractor2d* New();
    vtkTypeMacro(CInteractor2d, vtkInteractorStyleTrackballCamera);
    virtual void OnLeftButtonDown();
    inline virtual void OnRightButtonDown() {}
    inline virtual void OnRightButtonUp() {}
};
  
}

#endif
