//Low pass butterworth filter order=1 
//ref: http://www.schwietering.com/jayduino/filtuino/

#ifndef LP_FILTER_H
#define LP_FILTER_H

/*
 * cutoff 2.5hz, samplerate 333hz
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
      v[1] = (2.304624602137467573e-2 * x)
         + (0.95390750795725065547 * v[0]);
      return 
         (v[0] + v[1]);
    }
};

#endif



