#define _USE_MATH_DEFINES

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <complex>
#include <fstream>

#include <iostream>

using complex = std::complex<float>;

void DFT1D(const uint32_t& N, const complex *in, complex *out)
{
    for (int k = 0; k < N; ++k) {
        out[k] = 0;
        for (int n = 0; n < N; ++n) {
            double w = 2 * M_PI * n * k / N;
            out[k] += in[n] * complex(cos(w), -sin(w));
        }
    }
}

void DFT2D(const uint32_t& N, const complex *in, complex *out)
{
    // process the rows
    for (uint32_t i = 0; i < N; i++) {
        DFT1D(N, in + i * N, out + i * N);
    }

    // process the columns
    complex ca[N], cb[N];
    for (uint32_t i = 0; i < N; i++) {
        for (uint32_t j = 0; j < N; j++) {
            ca[j] = out[j * N + i]; // extract column with index j
        }
        DFT1D(N, ca, cb); // perform 1D DFT on this column
        for (uint32_t j = 0; j < N; j++) {
            out[j * N + i] = cb[j]; // store result back in the array
        }
    }
}

void SaveToPPM(const char* filename, const uint32_t& size, const complex* c)
{
	fprintf(stderr, "in save to ppm\n"); fflush(stderr);
	std::ofstream ofs;
	ofs.open(filename, std::ios::binary);

	float max = 0;
	for (uint32_t i = 0; i < size * size; ++i) {
		if (c[i].real() > max) max = c[i].real();
		if (c[i].imag() > max) max = c[i].imag();
	}

	ofs << "P5\n" << size << " " << size << "\n255\n";
	for (uint32_t j = 0; j < size; ++j) {
		for (uint32_t i = 0; i < size; ++i) {
			float val = std::abs(c[j * size + i].real());
			ofs << (unsigned char)(val * 255 / max);
		}	
	}
	
	ofs.close();
}

int main()
{
	std::cerr << "hello\n";
	
	const uint32_t nsamples = 1000;
	complex in[nsamples * nsamples];
	float wavelength = 200;
	float offset = 0;
	float amplitude = 1.0f;
	
	for (uint32_t j = 0; j < nsamples; ++j) {
		for (uint32_t i = 0; i < nsamples; ++i) {
			in[j * nsamples + i] = complex(amplitude * sin(2 * M_PI * i / wavelength + offset), 0);
		}
	}
	
	SaveToPPM("./data.ppm", nsamples, in);
	return 0;
	
	//complex out[nsamples * nsamples];
	//DFT1D(nsamples, in, out);
	//fprintf(stdout, "%s, %s, %s, %s\n", "index", "real", "out.real", "out.imag");
	//for (uint32_t i = 0; i < nsamples; ++i) {
	//	fprintf(stdout, "%d, %f, %f, %f\n", i, in[i].real(), out[i].real(), out[i].imag());
	//}
	//fflush(stdout);
	
	//return 0;
}