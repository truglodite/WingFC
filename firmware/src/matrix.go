package main

// --- Simple Matrix Library for the Kalman Filter Library ---

// Matrix represents a simple 2D float64 matrix.
type Matrix struct {
	rows, cols int
	data       []float64
}

// NewMatrix creates a new matrix of the given size.
func NewMatrix(rows, cols int) *Matrix {
	return &Matrix{
		rows: rows,
		cols: cols,
		data: make([]float64, rows*cols),
	}
}

// At returns the value at a specific row and column.
func (m *Matrix) At(r, c int) float64 {
	return m.data[r*m.cols+c]
}

// Set sets the value at a specific row and column.
func (m *Matrix) Set(r, c int, val float64) {
	m.data[r*m.cols+c] = val
}

// Identity returns a new identity matrix of size n x n.
func Identity(n int) *Matrix {
	m := NewMatrix(n, n)
	for i := 0; i < n; i++ {
		m.Set(i, i, 1.0)
	}
	return m
}

// Add returns the sum of two matrices.
func (m *Matrix) Add(other *Matrix) *Matrix {
	res := NewMatrix(m.rows, m.cols)
	for i := 0; i < len(m.data); i++ {
		res.data[i] = m.data[i] + other.data[i]
	}
	return res
}

// Subtract returns the difference of two matrices.
func (m *Matrix) Subtract(other *Matrix) *Matrix {
	res := NewMatrix(m.rows, m.cols)
	for i := 0; i < len(m.data); i++ {
		res.data[i] = m.data[i] - other.data[i]
	}
	return res
}

// Multiply returns the product of two matrices.
func (m *Matrix) Multiply(other *Matrix) *Matrix {
	if m.cols != other.rows {
		panic("Matrix dimensions for multiplication are incompatible")
	}
	res := NewMatrix(m.rows, other.cols)
	for i := 0; i < m.rows; i++ {
		for j := 0; j < other.cols; j++ {
			sum := 0.0
			for k := 0; k < m.cols; k++ {
				sum += m.At(i, k) * other.At(k, j)
			}
			res.Set(i, j, sum)
		}
	}
	return res
}

// Transpose returns the transpose of the matrix.
func (m *Matrix) Transpose() *Matrix {
	res := NewMatrix(m.cols, m.rows)
	for i := 0; i < m.rows; i++ {
		for j := 0; j < m.cols; j++ {
			res.Set(j, i, m.At(i, j))
		}
	}
	return res
}

// Inverse returns the inverse of a square matrix using Gauss-Jordan elimination.
func (m *Matrix) Inverse() *Matrix {
	if m.rows != m.cols {
		panic("Inverse requires a square matrix")
	}
	n := m.rows

	// Build augmented matrix [M | I]
	aug := make([][]float64, n)
	for i := 0; i < n; i++ {
		aug[i] = make([]float64, 2*n)
		for j := 0; j < n; j++ {
			aug[i][j] = m.At(i, j)
		}
		for j := 0; j < n; j++ {
			if i == j {
				aug[i][n+j] = 1.0
			} else {
				aug[i][n+j] = 0.0
			}
		}
	}

	// Forward elimination / Gauss-Jordan with partial pivoting
	for col := 0; col < n; col++ {
		// Find pivot
		pivot := col
		maxAbs := abs(aug[col][col])
		for r := col + 1; r < n; r++ {
			if abs(aug[r][col]) > maxAbs {
				maxAbs = abs(aug[r][col])
				pivot = r
			}
		}
		if maxAbs < 1e-12 {
			panic("Matrix is singular or nearly singular and cannot be inverted")
		}
		// Swap rows if needed
		if pivot != col {
			aug[col], aug[pivot] = aug[pivot], aug[col]
		}

		// Normalize pivot row
		pivVal := aug[col][col]
		for j := 0; j < 2*n; j++ {
			aug[col][j] /= pivVal
		}

		// Eliminate other rows
		for r := 0; r < n; r++ {
			if r == col {
				continue
			}
			factor := aug[r][col]
			if factor == 0 {
				continue
			}
			for j := 0; j < 2*n; j++ {
				aug[r][j] -= factor * aug[col][j]
			}
		}
	}

	// Extract inverse from augmented matrix
	res := NewMatrix(n, n)
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			res.Set(i, j, aug[i][n+j])
		}
	}
	return res
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
