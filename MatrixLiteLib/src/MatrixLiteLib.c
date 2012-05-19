#include <malloc.h>
#include "MatrixLiteLib.h"
#ifndef NULL
	#define NULL 0
#endif
#define ERRORCHECK		/* you should only define this for debugging purposes, this adds a touch of overhead to every function, you should just be damn confident your not fucking up */
#undef ERRORCHECK

/* -ffunction-sections -Wl,-gc-sections <-- used these during building */

/* Private Support Functions */
bool InitializeMatrixData(LPMATRIX matrix, tinyint rows, tinyint cols)
{
	matrix->rows = rows;
	matrix->cols = cols;
	matrix->length = rows*cols;
	matrix->data = (float *)calloc(rows*cols,sizeof(float));	/* memset is unnecessary since calloc does it for you */
	if (matrix->data == NULL)
		return false;
	return true;
}
void MLmemcpy(void *d, const void *source, size_t len)	/* used in place of memcpy so I could ditch string.h */
{
	char *dest = (char *)d;
	const char *src = (const char *)source;

	while (len-- > 0)
		*dest++ = *src++;
}

/* Public Support Functions */
LPMATRIX CreateIdentityMatrix(tinyint rows)
{
	LPMATRIX matrix = (LPMATRIX)malloc(sizeof(_MATRIX));
    tinyint r = 0;
	InitializeMatrixData(matrix,rows,rows);
	
	for (r = 0; r < rows; r++)
		MatrixSet(matrix,r,r,1);
	return matrix;	
}
LPMATRIX CreateMatrices(tinyint rows, tinyint cols, tinyint number)
{
	LPMATRIX matrices = (LPMATRIX)calloc(number,sizeof(_MATRIX));
	tinyint s = 0;
	for (s = 0; s < number; s++)
		InitializeMatrixData(&matrices[s],rows, (cols == 0 ? rows : cols));
	return matrices;
}
void FreeMatrices(LPMATRIX matrices, tinyint number)
{
	tinyint s = 0;
	for (s = 0; s < number; s++)
		if (matrices[s].data)
			free(matrices[s].data);		/* no other way to do it since the internal data poitner must be freed (blah @ no classes!) */
	free(matrices);
}
LPMATRIX MatrixCopy(LPMATRIX src)
{
	LPMATRIX mat = CreateMatrix(src->rows,src->cols);
	MLmemcpy((void*)mat->data,(void*)src->data,src->length*sizeof(float));
	return mat;
}



/* Get and Set Methods */
float MatrixGet(const LPMATRIX matrix, tinyint row, tinyint col) { return matrix->data[(row*matrix->cols)+col]; }
bool MatrixSet(LPMATRIX matrix, tinyint row, tinyint col, float data) 
{ 
	if ((row*matrix->cols)+col < matrix->length)
		matrix->data[(row*matrix->cols)+col] = data; 
	else
		return false;
	return true;
}
void MatrixSetAll(LPMATRIX matrix, float value) 
{
	tinyint s = 0;
	for (s = 0; s < matrix->length; s++)
		matrix->data[s] = value;
}

/* Basic Operations */
LPMATRIX MatrixAddSub(const LPMATRIX first, const LPMATRIX second, bool bAdd)
{
	#ifdef ERRORCHECK
		if (first->rows != second->rows && first->cols != second->cols)
			return NULL;
	#endif

	LPMATRIX mat = CreateMatrix(first->rows,second->cols);
	tinyint s = 0;
	for (s = 0; s < first->length; s++)
		mat->data[s] = first->data[s] + (bAdd ? second->data[s] : -second->data[s]);
	return mat;
}

LPMATRIX MatrixMultiply(const LPMATRIX first, LPMATRIX second)			/* a lot can be done here since its square matrices, but we dont wanna sacrifice memory for speed dont forget that */
{																		/* but at the same time the damn thing isn't exactly a speed demon so lets just be creative */
	#ifdef ERRORCHECK
		if (first->rows != second->cols)
			return NULL;
	#endif
	LPMATRIX mat = CreateMatrix(first->rows,second->cols);
	tinyint f = 0, s = 0, col = 0, row = 0;
	for (f = 0, col = 0, row = 0; f < first->length; f++, col++)
	{
		if (col == first->cols)
		{
			col = 0;
			row++;
		}

		for (s = 0; s < second->cols; s++)
			mat->data[(row*mat->cols)+s] += first->data[f]*second->data[col*second->cols+s];
	}
	return mat;
}

LPMATRIX MatrixTranspose(const LPMATRIX matrix)
{
	/* Our list form doesn't lend itself well to transposition (or at least I can't come up with a good pattern to work off of yet
	   So a easy solution is to just do the whole matrix (especially since they arent all square) */
	LPMATRIX mat = CreateMatrix(matrix->cols,matrix->rows);
    tinyint r = 0, c = 0;
	if (matrix->cols == 1 || matrix->rows == 1)	 /* easy transposal for vectors */
	{
		MLmemcpy((void*)mat->data,(void*)matrix->data,matrix->length*sizeof(float));
		return mat;
	}
	
	/*	Square matrices would be a bit easier since we could only do ~2/3rds
		(but since division is a sucky operation its probably easier to do the whole damn matrix) */

	for (r = 0; r < matrix->rows; r++)
		for (c = 0; c < matrix->cols; c++)
			MatrixSet(mat,c,r,MatrixGet(matrix,r,c));	/* i really hate to do it this way but i jsut dont see another way at this time... I will optimize it later */
	return mat;
}
/* Implementation of CholeskyCrout ganked from http://www.gpstk.org/doxygen/MatrixFunctors_8hpp-source.html#l00658
   More ganks http://www.gpstk.org/doxygen/MatrixOperators_8hpp-source.html#l00544
   Modified for a c environment */
LPMATRIX MatrixInverse(const LPMATRIX matrix)
{
#ifdef ERRORCHECK
	if (matrix->rows != matrix->cols)
		return NULL;
#endif

	LPMATRIX L = CreateIdentityMatrix(matrix->rows);		/* we will operate on this matrix */
	LPMATRIX D = CreateMatrix(matrix->rows, matrix->rows);	/* The matrix so that we don't have to use sqrt */
	tinyint i = 0, j = 0, k = 0;
    LPMATRIX transpose = NULL, step = NULL, result = NULL, LI = NULL;

	float sum = 0;
	for (j = 0; j < matrix->rows; j++)						/* calculates our L triangular matrix */
	{
		sum = MatrixGet(matrix,j,j);
		for (k = 0; k < j; k++) 
			sum -= MatrixGet(L,j,k) * MatrixGet(L,j,k) * MatrixGet(D,k,k);
		MatrixSet(D,j,j,sum);
	
		for (i = j+1; i < matrix->rows; i++)
		{
			sum = MatrixGet(matrix,i,j);
			for (k = 0; k < j; k++) 
				sum -= MatrixGet(L,i,k)*MatrixGet(L,j,k)*MatrixGet(D,k,k);
			MatrixSet(L,i,j,sum/MatrixGet(D,j,j));
		}
	}

	/* calculates the inverse of L */
	LI = CreateMatrix(matrix->rows, matrix->rows);
	for (i = 0; i < matrix->rows; i++)
	{
		MatrixSet(LI,i,i, 1.0);
		for (j = 0; j < i; j++)
		{
			sum = 0.0;
			for (k = 0; k <= i; k++) 
				sum += MatrixGet(L,i,k)*MatrixGet(LI,k,j);				/* interesting note uchar will never be < 0 since it wraps to 255 so we modify it this time... */
			MatrixSet(LI,i,j,-sum*MatrixGet(LI,i,i));					/* thus this loop got fliped around since I dont see a reason it would matter */
		}
	}
	/* calculates the inverse of D (which we will do in place for memories sake, and in list form directly so its speedy) */
	for (i = 0; i < D->length; i++)
		if (D->data[i] != 0)
			D->data[i] = 1/D->data[i];

	/* Now this is the magic, LI*DI*LI` = A` */
	transpose = MatrixTranspose(LI);
	step = MatrixMultiply(transpose,D);	/* D is inverted in place earlier so there is no DI */
	result = MatrixMultiply(step,LI);

	FreeMatrix(transpose);
	FreeMatrix(step);
	FreeMatrix(D);
	FreeMatrix(LI);
	FreeMatrix(L);
	return result;
}
