#ifndef __vtkSurfaceFitter_h
#define __vtkSurfaceFitter_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkDRCFiltersModule.h>

class vtkPlane;
class vtkPolygon;
class vtkRectd;

class VTKDRCFILTERS_EXPORT vtkSurfaceFitter : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkSurfaceFitter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkSurfaceFitter *New();


  vtkSetMacro(MaxError, double);
  vtkGetMacro(MaxError, double);

  vtkSetMacro(MaxAngle, double);
  vtkGetMacro(MaxAngle, double);

  vtkSetMacro(SearchRadius, double);
  vtkGetMacro(SearchRadius, double);

  vtkSetMacro(MinimumNumberOfPoints, int);
  vtkGetMacro(MinimumNumberOfPoints, int);


  static void ComputePlane(vtkPolyData* polyData, vtkPlane* plane);
  static void ComputeConvexHull(vtkPolyData* polyData, vtkPlane* plane, vtkPolyData* convexHull);
  static void ComputeMinimumAreaRectangleFit(vtkPolyData* polyData, vtkPlane* plane, vtkRectd* rectangle);
  static void ComputeKnownSizeRectangleFit(vtkPolyData* polyData, vtkPlane* plane, vtkRectd* rectangle);


protected:

  double MaxError;
  double MaxAngle;
  double SearchRadius;
  int MinimumNumberOfPoints;

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);


  vtkSurfaceFitter();
  virtual ~vtkSurfaceFitter();

private:
  vtkSurfaceFitter(const vtkSurfaceFitter&);  // Not implemented.
  void operator=(const vtkSurfaceFitter&);  // Not implemented.
};

#endif
