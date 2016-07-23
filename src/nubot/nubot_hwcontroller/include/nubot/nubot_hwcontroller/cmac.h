#ifndef CMAC_H
#define CMAC_H

#include <stdio.h>
#include <string>

class CMAC
{
public:
    CMAC(size_t IpSize, size_t OpSize, int c, int Qlevels, long MemSize);
    ~CMAC();

    void Calc(double ip[], double op[]);
    void Train(double delta[]);
    void SetInputRange(double *imin, double *imax);
    void SaveTable(std::string filename);
    bool LoadTable(std::string filename);
public:
    long *addrs;
    double *IpRange,*Imax,*Imin;        // the range in which each element of IP is expected to fall
    int c;                 // the number of addresses returned
    long memsize;          // the total number of weights used by the network
    double** wts;         // the weights matrix
    double *op;            // the output of the network
    double beta;           // the learning rate for the LMS algorithm and should be in the range 0 to 2.
    int Qlevels;           // 单层量化等级 = (int)(iprange/width)
    double offset;         // 层间间隔    =  iprange/(Qlevels*c）
    double Qwidth;         // 量化单位长度 = (int)(iprange/width)
    bool use_hash;
    size_t IpSize,OpSize;
};

#endif // CMAC_H
