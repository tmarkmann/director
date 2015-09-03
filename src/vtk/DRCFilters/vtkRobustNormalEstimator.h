#ifndef __vtkRobustNormalEstimator_h
#define __vtkRobustNormalEstimator_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkDRCFiltersModule.h>

class VTKDRCFILTERS_EXPORT vtkRobustNormalEstimator : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkRobustNormalEstimator, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRobustNormalEstimator *New();

  vtkSetMacro(Radius, double);
  vtkGetMacro(Radius, double);

  vtkSetMacro(MaxEstimationError, double);
  vtkGetMacro(MaxEstimationError, double);

  vtkSetMacro(MaxCenterError, double);
  vtkGetMacro(MaxCenterError, double);

  vtkSetMacro(MaxIterations, int);
  vtkGetMacro(MaxIterations, int);

  vtkSetMacro(ComputeCurvature, bool);
  vtkGetMacro(ComputeCurvature, bool);

protected:

  bool ComputeCurvature;
  int MaxIterations;
  double Radius;
  double MaxEstimationError;
  double MaxCenterError;

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);


  vtkRobustNormalEstimator();
  virtual ~vtkRobustNormalEstimator();

private:
  vtkRobustNormalEstimator(const vtkRobustNormalEstimator&);  // Not implemented.
  void operator=(const vtkRobustNormalEstimator&);  // Not implemented.
};

#endif
