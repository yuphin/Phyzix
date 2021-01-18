
// Fixed SparseMatrix representation
StructuredBuffer<float> x;
RWStructuredBuffer<float> x_out;
StructuredBuffer<int> row_start;
StructuredBuffer<int> col_index;
StructuredBuffer<float> mat_values;
StructuredBuffer<float> rhs;

cbuffer CB : register(b0) {
	int N;
}

[numthreads(32, 1, 1)]
void CS(uint3 id : SV_DispatchThreadID) {
	int row = id.x;
	if (row >= N) {
		return;
	}
	float sig = 0.0;
	float diag;
	for (int j = row_start[row]; j < row_start[row + 1]; j++) {
		if (row != col_index[j]) {
			sig += mat_values[j] * x[col_index[j]];
		} else {
			diag = mat_values[j];
		}
	}
	x_out[row] = (rhs[row] - sig) * 1 / diag;

}