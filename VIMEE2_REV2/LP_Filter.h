//Low pass butterworth filter order=1 
//ref: http://www.schwietering.com/jayduino/filtuino/

#ifndef LP_FILTER_H
#define LP_FILTER_H

/*
 * cutoff 2.5hz, samplerate 500hz
 */
class  LP_filter
{
  public:
    LP_filter()
    {
      v[0]=0.0;
    }
  private:
    float v[2];
  public:
    float filt(float x) //class II 
    {
      v[0] = v[1];
      v[1] = (1.546629140310340489e-2 * x)
         + (0.96906741719379319022 * v[0]);
      return 
         (v[0] + v[1]);
    }
};

#endif



