#define _USE_MATH_DEFINES

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <fstream>

#include <iostream>
#include <complex>
#include <cassert>

using complex = std::complex<float>;

void DFT1D(const uint32_t& N, const complex* in, complex* out)
{
    for (int k = 0; k < N; ++k) {
        out[k] = 0;
        for (int n = 0; n < N; ++n) {
            double w = 2 * M_PI * n * k / N;
            out[k] += in[n] * complex(cos(w), -sin(w));
        }
    }
}

void iDFT1D(const int N, const complex* in, complex* out)
{
	for (int n = 0; n < N; ++n) {
        out[n] = 0;
        for (int k = 0; k < N; ++k) {
            double w = 2 * M_PI * n * k / N;
            out[n] += in[k] * complex(cos(w), sin(w));
        }
        out[n] /= N;
    }
}

template <typename OP>
void DFT2D(const uint32_t& N, const complex* in, complex*& out, OP op)
{
	fprintf(stderr, "Processing rows\n");
	fflush(stderr);
    // process the rows
    for (uint32_t i = 0; i < N; i++) {
        op(N, &in[N * i], &out[N * i]);
		fprintf(stdout, "\r%3d%%", int(100 * i / (float)N));
		fflush(stdout);
    }

    // process the columns
	fprintf(stderr, "\rProcessing columns\n");
	fflush(stderr);
    
    for (uint32_t i = 0; i < N; i++) {
        complex* ca = new complex[N];
		complex* cb = new complex[N];
		for (uint32_t j = 0; j < N; j++) {
            ca[j] = out[j * N + i]; // extract column with index j
        }
        op(N, ca, cb); // perform 1D DFT on this column
        for (uint32_t j = 0; j < N; j++) {
            out[j * N + i] = cb[j]; // store result back in the array
        }
		fprintf(stdout, "\r%3d%%", int(100 * i / (float)N));
		fflush(stdout);
		delete [] ca;
		delete [] cb;
	}
}

void SaveToPPM(const char* filename, const uint32_t& size, const complex* c)
{
	std::ofstream ofs;
	ofs.open(filename, std::ios::binary);

	float max = 0;
	for (uint32_t i = 0; i < size * size; ++i) {
		if (std::fabs(c[i].real()) > max) max = std::fabs(c[i].real());
		//if (std::fabs(c[i].imag()) > max) max = std::fabs(c[i].imag());
		//assert(c[i].real() >= 0);
	}
	std::cerr << "max " << max << std::endl;

	ofs << "P5\n" << size << " " << size << "\n255\n";
	for (uint32_t j = 0; j < size; ++j) {
		for (uint32_t i = 0; i < size; ++i) {
			//float val = std::max(
			//	std::fabs(c[j * size + i].real()),
			//	std::fabs(c[j * size + i].imag())
			//);
			//fprintf(stderr, "%f %f\n", std::fabs(c[j * size + i].real()),std::fabs(c[j * size + i].imag()));
			//fflush(stderr);
			//ofs << (unsigned char)(std::min(255.f, 255 * std::fabs(c[j * size + i].real())));
			ofs << (unsigned char)(std::min(255.f, std::fabs(c[j * size + i].real())));
		}
		//exit(0);
	}
	
	ofs.close();
}


#define N 256

complex* ReadPPM(const char* filename)
{
	std::ifstream ifs;
	ifs.open(filename, std::ios::binary);
	std::string header;
	ifs >> header;
	uint32_t width, height, dummy;
	ifs >> width >> height >> dummy;
	ifs.ignore();
	//std::cout << header << " " << width << " " << height << " " << dummy << std::endl;
	assert(width == height && width == N);
	
	complex *c = new complex[N * N];
	
	for (uint32_t i = 0; i < N * N; ++i) {
		unsigned char rgb[3];
		ifs.read((char*)rgb, 3);
		//fprintf(stderr, "%d ", (int)rgb[0]); fflush(stderr);
		//c[i] = complex(255 * i / (float)N, 0);//complex(rgb[0] / 255.f, 0);
		c[i] = complex(rgb[0], 0);
	}

	ifs.close();
	
	return c;
}

complex* CreateSquare(uint32_t size = 257)
{
	complex* c = new complex[size * size];
	for (uint32_t i = 0; i < size * size; ++i)
		c[i] = complex(0, 0);
	
	c[127 * size + 128] = complex(100, 0);
	
	return c;
}

int main()
{	
	float wavelength = 32;
	float offset = 0;
	float amplitude = 1.0f;
	
#if 0
	complex* test_in = new complex[N];
	complex* test_out = new complex[N];
	
	for (uint32_t i = 0; i < N; ++i) {
		test_in[i] = complex(amplitude * sin(2 * M_PI * i / wavelength + offset), 0);
	}
	
	DFT1D(N, test_in, test_out);
	iDFT1D(N, test_out, test_in);
	
	for (uint32_t i = 0; i < N; ++i) {
		fprintf(stderr, "%d %f %f\n", i,
			amplitude * sin(2 * M_PI * i / wavelength + offset), test_in[i].real());
		fflush(stderr);
	}
	
	delete [] test_in;
	delete [] test_out;

	return 0;
#endif	
	
#if 0
	complex* A = CreateSquare(N);
	//complex* A = new complex[N * N];
	//for (uint32_t j = 0; j < N; ++j) {
	//	for (uint32_t i = 0; i < N; ++i) {
	//		A[j * N + i] = complex(amplitude * sin(2 * M_PI * i / wavelength + offset), 0);
	//	}
	//}
#else
	complex* A = ReadPPM("./titi.ppm");//new complex[N * N];
#endif
	complex* B = new complex[N * N];

	// forward
	for (uint32_t y = 0; y < N; ++y) {
		DFT1D(N, A + y * N, B + y * N);
	}
	
	complex col_in[N], col_out[N];
	for (uint32_t x = 0; x < N; ++x) {
		for (uint32_t y = 0; y < N; ++y)
			col_in[y] = B[y * N + x];
		DFT1D(N, col_in, col_out);
		for (uint32_t y = 0; y < N; ++y)
			B[y * N + x] = col_out[y]; 
	}

#if 0
	// inverse
	for (uint32_t x = 0; x < N; ++x) {
		for (uint32_t y = 0; y < N; ++y)
			col_in[y] = B[y * N + x];
		iDFT1D(N, col_in, col_out);
		for (uint32_t y = 0; y < N; ++y)
			B[y * N + x] = col_out[y];
	}
	
	for (uint32_t y = 0; y < N; ++y) {
		iDFT1D(N, B + y * N, A + y * N); 
	}
#else
	for (uint32_t y = 0; y < N; ++y) {
		iDFT1D(N, B + y * N, A + y * N);
	}
	
	for (uint32_t x = 0; x < N; ++x) {
		for (uint32_t y = 0; y < N; ++y)
			col_in[y] = A[y * N + x];
		iDFT1D(N, col_in, col_out);
		for (uint32_t y = 0; y < N; ++y)
			A[y * N + x] = col_out[y]; 
	}
#endif

	//DFT2D(N, data, fourier, DFT1D);
	
	//complex* out = new complex[N * N];
	
	//DFT2D(N, fourier, out, iDFT1D);
	
	//SaveToPPM("./data.ppm", N, out);
	SaveToPPM("./fourier.ppm", N, A);

	delete [] B;
	delete [] A;
	//delete [] fourier;
	//delete [] out;
	
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