public class Neural_Network {
  private Matrix[] layers;      // Para a camada presente cada vetor armazena os valores dos neurônios
  private Matrix[] deepLayers;  // Todas camadas temporais serão agrupadas em um único vetor de matrizes
  private Matrix[] weights;     // Pesos e Bias de cada camada
  private Matrix vetValues;     // Vetor com todos os valores de pesos e bias
  private int[] T_;             // Tamanho de cada vetor de deepLayers
  private int[] presentLayers;  // Present Layer -> número de células para cada camada presente
  private int[] tempLayers;     // Temporal Layers -> número de camadas temporais passadas
  private int N_;               // Número de camadas
  private int NP = 0;           // Número de pesos e bias
  
  Neural_Network(int e, int s) {
    int[] values = {e, s};
    int[] tl = new int[values.length];
    start(values, tl);
  }
  
  Neural_Network(int e, int o, int s) {
    int[] values = {e, o, s};
    int[] tl = new int[values.length];
    start(values, tl);
  }
  
  Neural_Network(int[] values) {
    int[] tl = new int[values.length];
    start(values, tl);
  }
  
  Neural_Network(int[] values, int[] tl) {
    start(values, tl);
  }
  
  private void start(int[] values, int[] tl) {
    if(values.length != tl.length || values.length < 2) {
      println("ERRO" + " no construtor Neural_Network(int[] values, int[] tl) é esperado vetores de tamanho iguais.");
      return;
    }
    this.presentLayers = values.clone();
    this.tempLayers = tl.clone();
    N_ = presentLayers.length;
    T_ = new int[N_];
    this.layers = new Matrix[N_];
    this.deepLayers = new Matrix[N_];
    this.weights = new Matrix[N_ - 1];
    int v1, v2;
    for(int i = 0; i < N_; i++) {
      T_[i] = presentLayers[i]*(tempLayers[i] + 1) + 1; // +1 => neurônio adicional igual a 1 para o cálculo dos bias
      layers[i] = new Matrix(presentLayers[i], 1);
      deepLayers[i] = new Matrix(T_[i], 1);
    }
    for(int i = 0; i < N_ - 1; i++) {
      v1 = presentLayers[i+1];
      v2 = T_[i];
      weights[i] = new Matrix(v1, v2);
      NP += v1*v2;
    }
    this.vetValues = new Matrix(NP, 1);
  }
  
  void debuga() {
    for(int i = 0; i < N_-1; i++) 
    //println(i + " T[0] = " + T_[i] + " -> " + presentLayers[i] + "*(" + tempLayers[i] + " + 1) + 1");
    //println("layers[i].getNumLinhas() = " + layers[i].getNumLinhas());
    println("W[" + i + "].m = " + weights[i].getNumLinhas() + " n = " + weights[i].getNumColunas());
    println();
  }

  public void feedforward() {
    for(int i = 0; i < N_ - 2; i++) {
      //println("Camada " + i);
      //deepLayers[i].printMatrix();
      deepLayers[i].moveDown(layers[i]);
      deepLayers[i].set(deepLayers[i].getNumLinhas() - 1, 0, 1);
      layers[i+1] = Matrix.mult(weights[i], deepLayers[i]);
      reLu(layers[i+1]);
      //deepLayers[i].printMatrix(); println();
    }
    deepLayers[N_-2].moveDown(layers[N_-2]);
    deepLayers[N_ - 2].set(deepLayers[N_ - 2].getNumLinhas() - 1, 0, 1);
    layers[N_ - 1] = Matrix.mult(weights[N_-2], deepLayers[N_-2]);
    //layers[N_ - 1].printMatrix();println();
  }

  public void sigmoid(Matrix mat) {
    float[] vet;
    if (mat.mat.length <= 1) vet = mat.mat[0];
    else {
      mat.transposta();
      vet = mat.mat[0];
    }
    for (int i = 0; i < vet.length; i++) {
      vet[i] = sigmoid(vet[i]);
    }
    mat.transposta();
  }

  public void sigmoid(float[] vet) {
    for (int i = 0; i < vet.length; i++) {
      vet[i] = sigmoid(vet[i]);
    }
  }

  public float sigmoid(float x) {
    return 1.0 / (1.0 + exp(-x));
  }
  
  public void reLu(Matrix mat) {
    for(int i = 0; i < mat.getNumLinhas(); i++) {
      reLu(mat.mat[i]);
    }
  }
  
  public void reLu(float[] vet) {
    for (int i = 0; i < vet.length; i++) {
      vet[i] = reLu(vet[i]);
    }
  }
  
  public float reLu(float x) {
    return max(0, x);
  }
  
  public void updateValues(Matrix vet) {
    if(vet.getNumLinhas() != NP) {
      println("Error in updateValues");
      return;
    }
    int ind = 0;
    for(int i = 0; i < N_-1; i++) {
      for(int m = 0; m < weights[i].getNumLinhas(); m++)
        for(int n = 0; n < weights[i].getNumColunas(); n++) {
          weights[i].set(m, n, vet.get(ind, 0));
          ind++;
        }  
    }
  }
  
  public void updateValues(float vet[]) {
    Matrix mat = new Matrix(vet);
    mat.transposta();
    updateValues(mat);
  }
  
  public void updateEntrance(Matrix entrance) {
    this.layers[0] = entrance;
  }

  public void updateOutput(Matrix output) {
    this.layers[N_-1] = output;
  }

  public void updateHiddenWeights(Matrix[] weights) {
    this.weights = weights;
  }
  
  public void biasAndWeightsRandom() {
    for(int i = 0; i < weights.length; i++) weights[i].randomFillNormal();
  }

  public void allRandom() {
    for(int i = 0; i < layers.length; i++) layers[i].randomFill();
    for(int i = 0; i < weights.length; i++) weights[i].randomFillNormal();
    for(int i = 0; i < deepLayers.length; i++) deepLayers[i].randomFill();
  }
  
  public Matrix join(Matrix m1, Matrix m2, float mutation) {
    Matrix vet = m1.copy();
    if (m1.getNumLinhas() == m2.getNumLinhas() && m1.getNumColunas() == 1) {
      for (int i = 0; i < m1.getNumLinhas(); i++) {
        if (random(1.0) < mutation) {
          if (random(1.0) <= 0.8) vet.mat[i][0] += vet.nextGaussian();
          else vet.mat[i][0] = random(-1.0, 1.0);
        } else {
          if (random(1.0) < 0.5) vet.mat[i][0] = m2.mat[i][0];
        }
      }
    }
    return vet.copy();
  }

  public Matrix join(Matrix m1, Matrix m2, float mutation, float dP) {
    Matrix vet = m1.copy();
    if (m1.getNumLinhas() == m2.getNumLinhas() && m1.getNumColunas() == 1) {
      for (int i = 0; i < m1.getNumLinhas(); i++) {
        if (random(1.0) < mutation) {
          if (random(1.0) <= 0.8) vet.mat[i][0] += dP*vet.nextGaussian();
          else vet.mat[i][0] = random(-dP, dP);
        } else {
          if (random(1.0) < 0.5) vet.mat[i][0] = m2.mat[i][0];
        }
      }
    }
    return vet.copy();
  }
  
  public Neural_Network get() {
    Neural_Network net = new Neural_Network(presentLayers, tempLayers);
    net.updateValues(vetValues.copy());
    return net;
  }
  
  public void setVetValues(Matrix vet) {
    if (vet.getNumLinhas() == vetValues.getNumLinhas() && vet.getNumColunas() == 1) this.vetValues = vet;
    else println("Error in setVetValues()");
  }
  
  public int getEntranceLength() { 
    return this.layers[0].getNumLinhas();
  }
  
  public int getHiddenLength() { 
    return N_-2;
  }
  
  public int getOutputLength() { 
    return this.layers[N_-1].getNumLinhas();
  }
  
  public Matrix getOutput() { 
    return this.layers[N_-1];
  }
  
  public float[] getOutputVet() {
    float[] vet = new float[this.layers[N_-1].getNumLinhas()];
    for (int i = 0; i < this.layers[N_-1].getNumLinhas(); ) { 
      vet[i] = this.layers[N_-1].mat[i][0]; 
      i++;
    }
    return vet;
  }
  
  public Matrix getVetValues() { 
    return this.vetValues;
  }
  
  public int getVetValuesLength() { 
    return this.vetValues.getNumLinhas();
  }
}
