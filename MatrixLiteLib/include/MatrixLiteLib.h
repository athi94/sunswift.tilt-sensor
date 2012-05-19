/*
	MatrixLiteLib.h
	A collection of functions designed to work handle
	Simple operations on a matrix with a balance of speed
	and simplicity such that it does not use much memory
*/
#ifndef __MATRIXLIB_H__
#define __MATRIXLIB_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Create Matrix Macros */
#define CreateMatrix(rows, cols) CreateMatrices(rows, cols, 1)				/* Why have two functions when you can have one */
#define FreeMatrix(matrix) FreeMatrices(matrix,1)							/* Frees a single matrix, no sure adding over head */

/* Vector Matrix Macros  */
#define CreateVector(rows)				CreateMatrix(rows,1)
#define CreateVectors(rows,number)		CreateMatrices(rows,1,number)
#define FreeVector(vector)				FreeMatrix(vector)
#define FreeVectors(vectors, number)	FreeMatrices(vectors, number)

/* Operation Macros */
#define MatrixAdd(one, two) MatrixAddSub(one,two,true);							/* Ease of use to add two matrices */
#define MatrixSub(one, two) MatrixAddSub(one,two,false);						/* Ease of use to subtract two matrices */

/* Typedefs */
#ifndef __cplusplus				/* Take this out when were compiling in c only */
typedef unsigned char bool;

#define false 0
#define true 0
#endif
typedef unsigned char tinyint;			/* used for small integers (saves some memory) */

/* Matrix structure used for all operations */
typedef struct _MATRIX
{
	tinyint rows;		/* holds the number of rows  */
	tinyint cols;		/* holds the number of cols */
	tinyint length;		/* adds 8bits, but saves us a multiplication in a few places */
	float *data;		/* matrix data */
} _MATRIX, *LPMATRIX;

/* Matrix Support Functions */
extern LPMATRIX CreateIdentityMatrix(tinyint rows);							/* returns an identity matrix */
extern LPMATRIX CreateMatrices(tinyint rows, tinyint cols, tinyint number);	/* returns an array of initialized matrices */
extern void FreeMatrices(LPMATRIX matrices, tinyint number);				/* frees all the matrices in an array */
extern LPMATRIX MatrixCopy(LPMATRIX src);									/* copies a matrix into a new matrix */ 

/* Set and Get Functions */
extern float MatrixGet(const LPMATRIX matrix, tinyint row, tinyint col);		/* returns a value at a given column and row (0 based) */
extern bool MatrixSet(LPMATRIX matrix, tinyint row, tinyint col, float data);	/* sets data at a given column and row (0 based) */
extern void MatrixSetAll(LPMATRIX matrix, float value);						/* sets all values in a matrix to the same value */

/* Operation Functions */
extern LPMATRIX MatrixAddSub(const LPMATRIX one, const LPMATRIX two, bool bAdd);/* adds or subtracts two matrices */
extern LPMATRIX MatrixMultiply(const LPMATRIX first, LPMATRIX second);			/* multiplies two matrices together */

/* Higher Level Functions */
extern LPMATRIX MatrixTranspose(const LPMATRIX matrix);							/* returns a new transposed matrix (works on non square) */
extern LPMATRIX MatrixInverse(const LPMATRIX matrix);							/* returns the inverse of a matrix, very fun operations */
#ifdef __cplusplus
}
#endif
#endif
