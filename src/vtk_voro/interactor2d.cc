/*
 * File name: interactor2d.cpp
 * Date:      2012-11-22 18:31:20 +0100
 * Author:    Miroslav Kulich
 */

#include "interactor2d.h"

#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>

using namespace imr;
vtkStandardNewMacro(CInteractor2d);

void CInteractor2d::OnLeftButtonDown()
{
  this->FindPokedRenderer(this->Interactor->GetEventPosition()[0],
                          this->Interactor->GetEventPosition()[1]);

  if (this->CurrentRenderer == NULL) {
    return;
  }
  if (!this->Interactor->GetControlKey()) {
    this->StartPan();
  }
}

