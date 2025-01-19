// UPDATE IN 19/12/2020

import java.util.*;

static public class Matrix {
  private int linhas  = 1;
  private int colunas = 1;
  public  float[][] mat;
  private Random normal = new Random();
  
  Matrix() {
    mat = new float[linhas][colunas];
  }
  
  Matrix(float[][] m) {
    if(m.length > 0) {
      linhas = m.length;
      colunas = m[0].length;
      this.mat = m;
    }
  }
  
  Matrix(float[] m) {
    linhas = 1;
    colunas = m.length;
    this.mat = new float[linhas][colunas];
    this.mat[0] = m;
  }
  
  Matrix(int n) {
    this.mat = new float[n][n];
    this.linhas = n;
    this.colunas = n;
  }
  
  Matrix(int l, int c) {
    this.mat = new float[l][c];
    this.linhas = l;
    this.colunas = c;
  }
  
  public void randomFill() {
    for(int i = 0; i < linhas; i++) {
      for(int j = 0; j < colunas; j++) {
        this.mat[i][j] = (float) Math.random();
      }
    }
  }
  
  public void randomFill(float min, float max) {
    for(int i = 0; i < linhas; i++) {
      for(int j = 0; j < colunas; j++) {
        this.mat[i][j] = min + (max - min) * (float) Math.random();
      }
    }
  }
  
  public void randomFill(float max) {
    for(int i = 0; i < linhas; i++) {
      for(int j = 0; j < colunas; j++) {
        this.mat[i][j] = max * (float) Math.random();
      }
    }
  }
  
  public void randomFillNormal(float desvio, float media) {
    for(int i = 0; i < linhas; i++) {
      for(int j = 0; j < colunas; j++) {
        this.mat[i][j] = (float) normal.nextGaussian() * desvio + media;
      }
    }
  }
  
  public void randomFillNormal(float desvio) {
    for(int i = 0; i < linhas; i++) {
      for(int j = 0; j < colunas; j++) {
        this.mat[i][j] = (float) normal.nextGaussian() * desvio;
      }
    }
  }
  
  public void randomFillNormal() {
    for(int i = 0; i < linhas; i++) {
      for(int j = 0; j < colunas; j++) {
        this.mat[i][j] = (float) normal.nextGaussian();
      }
    }
  }
  
  public float nextGaussian() {
    return (float) normal.nextGaussian();
  }
  
  public void printMatrix() {
    //println(linhas, colunas);
    for(int i = 0; i < linhas; i++) {
      println();
      for(int j = 0; j < colunas; j++) {
        print(nf(mat[i][j], 3, 2) + " \t");
      }
    }
    println();
  }
  
  static void printMatrix(float[][] mat) {
    for(int i = 0; i < mat.length; i++) {
      println();
      for(int j = 0; j < mat[0].length; j++) {
        print(nf(mat[i][j], 3, 2) + " \t");
      }
    }
    println();
  }
  
  static void printMatrix(Matrix A) {
    for(int i = 0; i < A.getNumLinhas(); i++) {
      println();
      for(int j = 0; j < A.getNumColunas(); j++) {
        print(nf(A.mat[i][j], 3, 2) + " \t");
      }
    }
    println();
  }
  
  public void transposta() {
    Matrix C = new Matrix(colunas, linhas);
    for(int i = 0; i < colunas; i++) {
      for(int j = 0; j < linhas; j++) {
        C.mat[i][j] = this.mat[j][i];
      }
    }
    mat = C.mat.clone();
    this.colunas = linhas;
    this.linhas  = mat.length;
  }
  
  static Matrix transposta(Matrix A) {
    Matrix C = new Matrix(A.getNumColunas(), A.getNumLinhas());
    for(int i = 0; i < A.getNumLinhas(); i++) {
      for(int j = 0; j < A.getNumColunas(); j++) {
        C.mat[i][j] = A.mat[j][i];
      }
    }
    C.colunas = C.linhas;
    C.linhas  = C.mat.length;
    return C;
  }
  
  public Matrix copy() {
    Matrix C = new Matrix(linhas, colunas);
    for(int i = 0; i < linhas; i++) {
      for(int j = 0; j < colunas; j++) {
        C.mat[i][j] = this.mat[i][j];
      }
    }
    return C;
  }
  
  static Matrix copy(Matrix A) {
    Matrix C = new Matrix(A.getNumLinhas(), A.getNumColunas());
    for(int i = 0; i < A.getNumLinhas(); i++) {
      for(int j = 0; j < A.getNumColunas(); j++) {
        C.mat[i][j] = A.mat[i][j];
      }
    }
    return C;
  }
  
  static Matrix sum(Matrix A, Matrix B) {
    if((A.getNumColunas() != B.getNumColunas()) || (A.getNumLinhas() != B.getNumLinhas())) return A;
    Matrix C = new Matrix(A.getNumColunas(), A.getNumLinhas());
    for(int i = 0; i < A.getNumLinhas(); i++) {
      for(int j = 0; j < A.getNumColunas(); j++) {
        C.mat[i][j] = A.mat[i][j] + B.mat[i][j];
      }
    }
    return C;
  }
  
  public void sum(Matrix A) {
    if((colunas != A.getNumColunas()) || (linhas != A.getNumLinhas())) return;
    for(int i = 0; i < A.getNumLinhas(); i++) {
      for(int j = 0; j < A.getNumColunas(); j++) {
        this.mat[i][j] += A.mat[i][j];
      }
    }
  }
  
  static Matrix sub(Matrix A, Matrix B) {
    if((A.getNumColunas() != B.getNumColunas()) || (A.getNumLinhas() != B.getNumLinhas())) return A;
    Matrix C = new Matrix(A.getNumColunas(), A.getNumLinhas());
    for(int i = 0; i < A.getNumLinhas(); i++) {
      for(int j = 0; j < A.getNumColunas(); j++) {
        C.mat[i][j] = A.mat[i][j] - B.mat[i][j];
      }
    }
    return C;
  }
  
  public void sub(Matrix A) {
    if((colunas != A.getNumColunas()) || (linhas != A.getNumLinhas())) return;
    for(int i = 0; i < A.getNumLinhas(); i++) {
      for(int j = 0; j < A.getNumColunas(); j++) {
        this.mat[i][j] += -A.mat[i][j];
      }
    }
  }
  
  static Matrix mult(Matrix A, float p) {
    Matrix C = new Matrix(A.getNumColunas(), A.getNumLinhas());
    for(int i = 0; i < A.getNumLinhas(); i++) {
      for(int j = 0; j < A.getNumColunas(); j++) {
        C.mat[i][j] = A.mat[i][j]*p;
      }
    }
    return C;
  }
  
  static Matrix mult(Matrix A, Matrix B) {
    if((A.getNumColunas() != B.getNumLinhas())) return A;
    Matrix C = new Matrix(A.getNumLinhas(), B.getNumColunas());
    for(int i = 0; i < A.getNumLinhas(); i++) {
      for(int j = 0; j < B.getNumColunas(); j++) {
        C.mat[i][j] = 0;
        for(int k = 0; k < A.getNumColunas(); k++) {
          C.mat[i][j] += A.mat[i][k] * B.mat[k][j];
        }
      }
    }
    return C;
  }
  
  public void mult(float p) {
    for(int i = 0; i < linhas; i++) {
      for(int j = 0; j < colunas; j++) {
        this.mat[i][j] = this.mat[i][j]*p;
      }
    }
  }
  
  public void mult(Matrix A) {
    if((colunas != A.getNumLinhas())) return;
    Matrix C = new Matrix(linhas, A.getNumColunas());
    for(int i = 0; i < linhas; i++) {
      for(int j = 0; j < A.getNumColunas(); j++) {
        C.mat[i][j] = 0;
        for(int k = 0; k < colunas; k++) {
          C.mat[i][j] += this.mat[i][k] * A.mat[k][j];
        }
      }
    }
    this.mat = C.mat;
    this.linhas = C.linhas;
    this.colunas = C.colunas;
  }
  
  public void updateValues(float[] values) { //Update 06/04/2020
    if(this.linhas > 1 && this.colunas > 1) return;
    int minV = min(this.linhas, values.length);
    if(this.linhas > this.colunas) {
      for(int i = 0; i < minV; i++) {
        this.mat[i][0] = values[i];
      }
    }
    else {
      for(int i = 0; i < minV; i++) {
        this.mat[0][i] = values[i];
      }
    }
  }
    
  public void updateValues(int[] values) { //Update 06/04/2020
    if(this.linhas > 1 && this.colunas > 1) return;
    int minV = min(this.linhas, values.length);
    if(this.linhas > this.colunas) {
      for(int i = 0; i < minV; i++) {
        this.mat[i][0] = values[i];
      }
    }
    else {
      for(int i = 0; i < minV; i++) {
        this.mat[0][i] = values[i];
      }
    }
  }
  
  public void swapLines(int l0, int l1) {
    if(!pertence(l0, 0) || !pertence(l1, 0)) return;
    float li;
    //l0 = l0 - 1;
    //l1 = l1 - 1;
    for(int i = 0; i < colunas; i++) {
      li = this.mat[l0][i];
      this.mat[l0][i] = this.mat[l1][i];
      this.mat[l1][i] = li;
    }
  }
  
  public void swapColumns(int c0, int c1) {
    if(!pertence(0, c0) || !pertence(0, c1)) return;
    float col;
    //c0 = c0 - 1;
    //c1 = c1 - 1;
    for(int i = 0; i < linhas; i++) {
      col = this.mat[i][c0];
      this.mat[i][c0] = this.mat[i][c1];
      this.mat[i][c1] = col;
    }
  }
  
  public void moveDown(Matrix mv) {
    if(mv.getNumColunas() == colunas) {
      int ds = mv.getNumLinhas();
      if(ds > getNumLinhas()) return;
      for(int i = linhas - 1; i >= ds; i--) {
        for(int j = 0; j < colunas; j++)
          mat[i][j] = mat[i-ds][j];
      }
      for(int i = 0; i < ds; i++) //setLine(seila, i); 
      setLine(mv.getLine(i), i);
    }
  }
  
  public Matrix getLine(int li) {
    if(!pertence(li, 0)) return null;
    Matrix lin = new Matrix(1, colunas);
    for(int i = 0; i < lin.getNumColunas(); i++) lin.mat[0][i] = this.mat[li][i];
    return lin;
  }
  
  public Matrix getColunm(int col) {
    if(!pertence(0, col)) return null;
    Matrix colunm = new Matrix(linhas, 1);
    for(int i = 0; i < colunm.getNumLinhas(); i++) colunm.mat[i][0] = this.mat[i][col];
    return colunm;
  }
  
  public int getNumLinhas() {
    return this.linhas;
  }
  
  public int getNumColunas() {
    return this.colunas;
  }
  
  public float get(int m, int n) {
    if(pertence(m, n)) return this.mat[m][n];
    else {
      println("Índices incorretos!");
      return -1;
    }
  }
  
  public boolean pertence(int m, int n) {
    return (m < getNumLinhas() && n < getNumColunas() && m >= 0 && n >= 0);
  }
  
  public void set(int m, int n, float v) {
    if(pertence(m, n)) this.mat[m][n] = v;
    else println("Índices incorretos!");
  }
  
  public void setLine(Matrix lin, int li) {
    if(getNumColunas() != lin.getNumColunas()) {
      println("ERROR in setLine(Matrix lin, int li)");
      return;
    }
    if(li < 0 || li >= getNumLinhas()) return;
    for(int i = 0; i < lin.getNumColunas(); i++) this.mat[li][i] = lin.mat[0][i];
  }
  
  public void setColunm(Matrix colunm, int col) {
    if(getNumLinhas() != colunm.getNumLinhas()) return;
    if(col < 0 || col >= getNumColunas()) return;
    for(int i = 0; i < colunm.getNumLinhas(); i++) this.mat[i][col] = colunm.mat[i][0];
  }
}
