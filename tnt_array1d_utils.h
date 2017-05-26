/*
*
* Template Numerical Toolkit (TNT)
*
* Mathematical and Computational Sciences Division
* National Institute of Technology,
* Gaithersburg, MD USA
*
*
* This software was developed at the National Institute of Standards and
* Technology (NIST) by employees of the Federal Government in the course
* of their official duties. Pursuant to title 17 Section 105 of the
* United States Code, this software is not subject to copyright protection
* and is in the public domain. NIST assumes no responsibility whatsoever for
* its use by other parties, and makes no guarantees, expressed or implied,
* about its quality, reliability, or any other characteristic.
*
*/

#ifndef TNT_ARRAY1D_UTILS_H
#define TNT_ARRAY1D_UTILS_H

#include <cstdlib>
#include <cassert>

namespace TNT
{





	/************************************************************************/
	/* append                                                                     */
	/************************************************************************/

	template <class T>
	Array1D<T>& operator*(const Array1D<T> &A, T B)
	{

		int n = A.dim1();

		Array1D<T> C(n);

		for (int i = 0; i < n; i++)
		{
			C[i] = A[i] * B;
		}
		return C;

	}


	template <class T>
	Array1D<T>&  operator*=(Array1D<T> &A, const T t)
	{
		int n = A.dim1();


		for (int i = 0; i < n; i++)
		{
			A[i] *= t;
		}

		return A;
	}


	template <class T>
	Array1D<T> operator/(const Array1D<T> &A, const T B)
	{
		int n = A.dim1();


		Array1D<T> C(n);

		for (int i = 0; i < n; i++)
		{
			C[i] = A[i] / B;
		}
		return C;

	}
	template <class T>
	Array1D<T>&  operator/=(Array1D<T> &A, const T B)
	{
		int n = A.dim1();


		for (int i = 0; i < n; i++)
		{
			A[i] /= B;
		}

		return A;
	}





template <class T>
T Array1D<T>::length(){

	int n = this->dim1();
	T ans = 0;
	for (int i = 0; i < n; i++)
	{
		ans += this->data_[i] * this->data_[i];
	}

	return sqrt(ans);

}

template <class T>
T Array1D<T>::sum(){

	int n = this->dim1();
	T ans = 0;
	for (int i = 0; i < n; i++)
	{
		ans += this->data_[i] ;
	}

	return ans;

}

template <class T>
Array1D<T>& Array1D<T>::crossProduct(const Array1D<T> &B)
{



	data_[0] = data_[1] * B[2] - B[1] * data_[2];
	data_[1] = data_[2] * B[0] - B[2] * data_[0];
	data_[2] = data_[0] * B[1] - B[0] * data_[1];

	return *this;

}


template <class T>
std::ostream& operator<<(std::ostream &s, const Array1D<T> &A)
{
    int N=A.dim1();

#ifdef TNT_DEBUG
	s << "addr: " << (void *) &A[0] << "\n";
#endif
    s <<"向量维数 = "<< N << "\n";
	s << "向量元素为 \n";
    for (int j=0; j<N; j++)
    {
       s<<"向量元素为 \n" << A[j] << "\n";
    }
    s << "\n";

    return s;
}

template <class T>
std::istream& operator>>(std::istream &s, Array1D<T> &A)
{
	int N;
	s >> N;

	Array1D<T> B(N);
	for (int i=0; i<N; i++)
		s >> B[i];
	A = B;
	return s;
}



template <class T>
Array1D<T> operator+(const Array1D<T> &A, const Array1D<T> &B)
{
	int n = A.dim1();

	if (B.dim1() != n )
		return Array1D<T>();

	else
	{
		Array1D<T> C(n);

		for (int i=0; i<n; i++)
		{
			C[i] = A[i] + B[i];
		}
		return C;
	}
}
template <class T>
Array1D<T> operator+=(const Array1D<T> &A, const Array1D<T> &B)
{
	int n = A.dim1();

	if (B.dim1() != n)
		return Array1D<T>();

	else
	{
		Array1D<T> C(n);

		for (int i = 0; i < n; i++)
		{
			A[i] += B[i];
		}
		return A;
	}
}


template <class T>
Array1D<T> operator-(const Array1D<T> &A, const Array1D<T> &B)
{
	int n = A.dim1();

	if (B.dim1() != n )
		return Array1D<T>();

	else
	{
		Array1D<T> C(n);

		for (int i=0; i<n; i++)
		{
			C[i] = A[i] - B[i];
		}
		return C;
	}
}


template <class T>
Array1D<T> operator*(const Array1D<T> &A, const Array1D<T> &B)
{
	int n = A.dim1();

	if (B.dim1() != n )
		return Array1D<T>();

	else
	{
		Array1D<T> C(n);

		for (int i=0; i<n; i++)
		{
			C[i] = A[i] * B[i];
		}
		return C;
	}
}



template <class T>
Array1D<T> operator/(const Array1D<T> &A, const Array1D<T> &B)
{
	int n = A.dim1();

	if (B.dim1() != n )
		return Array1D<T>();

	else
	{
		Array1D<T> C(n);

		for (int i=0; i<n; i++)
		{
			C[i] = A[i] / B[i];
		}
		return C;
	}
}









template <class T>
Array1D<T>&  operator+=(Array1D<T> &A, const Array1D<T> &B)
{
	int n = A.dim1();

	if (B.dim1() == n)
	{
		for (int i=0; i<n; i++)
		{
				A[i] += B[i];
		}
	}
	return A;
}




template <class T>
Array1D<T>&  operator-=(Array1D<T> &A, const Array1D<T> &B)
{
	int n = A.dim1();

	if (B.dim1() == n)
	{
		for (int i=0; i<n; i++)
		{
				A[i] -= B[i];
		}
	}
	return A;
}



template <class T>
Array1D<T>&  operator*=(Array1D<T> &A, const Array1D<T> &B)
{
	int n = A.dim1();

	if (B.dim1() == n)
	{
		for (int i=0; i<n; i++)
		{
				A[i] *= B[i];
		}
	}
	return A;
}



template <class T>
Array1D<T>&  operator/=(Array1D<T> &A, const Array1D<T> &B)
{
	int n = A.dim1();

	if (B.dim1() == n)
	{
		for (int i=0; i<n; i++)
		{
				A[i] /= B[i];
		}
	}
	return A;
}






} // namespace TNT

#endif
