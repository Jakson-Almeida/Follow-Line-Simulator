public class DNA {
  private Movimento robot  = new Movimento(0, 0, 0, 1, true);
  private Movimento oq     = new Movimento(0, 0, 0, 1, true);
  private int   numCiclos  = 50000; // Em microssegundos
  private float distMaxima = 15.8;  // Em metros
  private float velMax     = 3.0;   // m/s
  private float dP         = 0.5;    // Desvio Padrão
  private float erroAbsMax = 5.0;
  private float mutacao     = 0.015;
  private int   tamPopulacao = 100;
  private int   geracaoAtual  = 0;
  private int   pidSelecionado = 0;
  private int    numMelhores    = 2;
  private float[] velocidades    = new float[tamPopulacao];
  private float[] distPercorrida = new float[tamPopulacao];
  private float[] tempoGasto     = new float[tamPopulacao];
  private float[] somatorio      = new float[tamPopulacao];
  private float[] erro           = new float[tamPopulacao];
  private int[][] populacao       = new int [tamPopulacao][4];
  private int[][] melhoresPID      = new int [numMelhores][4];
  private float[] fitness           = new float[tamPopulacao];
  private int  [] somatorioOrdenado = new int  [tamPopulacao];
  private float pesoVel            = 0.30;
  private float pesoDist          =  1.00;
  private float pesoEstabilidade  =  0.30;
  private Neural_Network neuronMaster = new Neural_Network(1, 1, 1);
  private Matrix[] weightsAndBias = new Matrix[tamPopulacao];
  private Matrix[] bestWeights = new Matrix[numMelhores];
  private int amplitude;
  private PSO pso;
  
  DNA(Movimento r) {
    this.robot = r;
    //this.robot.neuron.allRandom();
    this.neuronMaster = this.robot.getNeural_Network();
    this.oq = r.get();
    this.velMax = r.getVelMax();
    this.amplitude = r.getAmplitude();
    this.pso = new PSO();
    this.pso.setSize(tamPopulacao);
    this.pso.setCoord(neuronMaster.getVetValuesLength());
    this.inicializa();
    this.gerarPopulacao();
  }
  
  DNA(Movimento r, int tempo, float dist) { // Tempo em microssegundos
    this.robot = r;
    //this.robot.neuron.allRandom();
    this.neuronMaster = this.robot.getNeural_Network();
    this.oq = r.get();
    this.velMax = r.getVelMax();
    this.amplitude = r.getAmplitude();
    if(tempo > 0) this.numCiclos = tempo/r.getVariacaoDeTempo();
    this.distMaxima = dist;
    this.pso = new PSO();
    this.pso.setSize(tamPopulacao);
    this.pso.setCoord(neuronMaster.getVetValuesLength());
    this.inicializa();
    this.gerarPopulacao();
    
    //Debugando
    //println("Começando a printar\n");
    //println("\n2º plano");
    //networkTest();
  }
  
  public void inicializa() {
    for (int i = 0; i < this.tamPopulacao; i++) {
      this.weightsAndBias[i] = new Matrix(neuronMaster.getVetValuesLength(), 1);
      this.weightsAndBias[i].randomFill(-dP, dP); //Update 07/04/2020
    } 
    for (int i = 0; i < this.numMelhores; i++) {
      this.bestWeights[i] = new Matrix(neuronMaster.getVetValuesLength(), 1);
      this.bestWeights[i].randomFill(-dP, dP);
    } 
  }
  
  public void gerarPopulacao() {
    for(char i = 0; i < tamPopulacao; i++) {
      populacao[i][0] = ceil(random(0, amplitude));
      populacao[i][1] = ceil(random(0, amplitude));
      populacao[i][2] = ceil(random(0, amplitude));
      populacao[i][3] = ceil(random(0, amplitude));
      weightsAndBias[i].randomFillNormal(0.2);
      for(int j = 0; j < neuronMaster.getVetValuesLength(); j++) pso.pop[i].pose[0].coord[j] = weightsAndBias[i].mat[j][0];
    }
  }
  
  public void testarPID(int i) {
    robot.reiniciarParametros();
    //robot = oq.get();
    robot.setPID(populacao[i].clone());
    for(int j = 0; j < numCiclos; j++) {
      if(robot.saiuDaLinha() || robot.getDistPercorrida() >= distMaxima) break;
      robot.lineControl();
    }
    erro[i] = robot.getErroAbsoluto() / ((float) robot.getTimeMicros() * erroAbsMax);
    if(erro[i] > 1.0) erro[i] = 1.0;
    tempoGasto[i] = robot.getTimeMicros() / 1000000.0;
    distPercorrida[i] = robot.getDistPercorrida();
    if(robot.getTimeMicros() > 0) {
      velocidades[i] = distPercorrida[i] * 1000000.0 / ((float) robot.getTimeMicros() * distMaxima);
    }
    else velocidades[i] = 0;
    somatorio[i] = (robot.getDistPercorrida()*pesoDist/distMaxima + velocidades[i]*pesoVel/velMax + (1.0 - erro[i])*pesoEstabilidade) / (pesoVel + pesoDist + pesoEstabilidade);
    
    // UPDATE IN 14/09/2020
    pso.pop[i].fitness = somatorio[i];
  }
  
  public void testarPID(int i, float distancia, long tempo) {
    distPercorrida[i] = distancia / distMaxima;
    if(tempo > 0) velocidades[i] = (robot.getDistPercorrida() * 1000000.0 / (float) tempo) / velMax;
    else velocidades[i] = 0;
    somatorio[i] = (distPercorrida[i]*pesoDist + velocidades[i]*pesoVel) / (pesoVel + pesoDist);
  }
  
  public void testarPopulacao() {
    for(int i = 0; i < tamPopulacao; i++) testarPID(i);
    somatorioOrdenado = ordenaIndiceDecrescente(somatorio).clone();
    for(int i = 0; i < tamPopulacao; i++) {
      try {
          this.fitness[i] = somatorio[somatorioOrdenado[i]];
      } catch (NullPointerException exception) {
          println("Erro ao calcular fitness");
      }
    }
  }
  
  public void testarRede(int i) {
    robot = oq.get();
    //robot.reiniciarParametros();
    robot.neuron.updateValues(weightsAndBias[i].copy());
    //robot.neuron.debuga();
    //robot.neuron.feedforward();
    for(int j = 0; j < numCiclos; j++) {
      if(robot.saiuDaLinha() || robot.getDistPercorrida() >= distMaxima) break;
      if(robot.getTimeMicros() > 2000000 && robot.getDistPercorrida() < 0.1) {
        somatorio[i] = 0;
        return;
      }
      robot.neuralControl();
    }
    //println(i + "_Dist: " + robot.getDistPercorrida());
    erro[i] = robot.getErroAbsoluto() / ((float) robot.getTimeMicros() * erroAbsMax);
    if(erro[i] > 1.0) erro[i] = 1.0;
    tempoGasto[i] = robot.getTimeMicros() / 1000000.0;
    distPercorrida[i] = robot.getDistPercorrida();
    if(robot.getTimeMicros() > 0) velocidades[i] = distPercorrida[i] * 1000000.0 / ((float) robot.getTimeMicros() * distMaxima);
    else velocidades[i] = 0;
    somatorio[i] = (robot.getDistPercorrida()*pesoDist/distMaxima + velocidades[i]*pesoVel/velMax + (1.0 - erro[i])*pesoEstabilidade) / (pesoVel + pesoDist + pesoEstabilidade);
    //somatorio[i] = robot.getDistPercorrida() * velocidades[i] * (1.0 - erro[i]);
    
    //UPDATE IN 14/09/2020
    pso.pop[i].fitness = somatorio[i];
  }
  
  public void networkTest() {
    for(int i = 0; i < tamPopulacao; i++) testarRede(i);
    somatorioOrdenado = ordenaIndiceDecrescente(somatorio).clone();
    for(int i = 0; i < tamPopulacao; i++) {
      try {
          this.fitness[i] = somatorio[somatorioOrdenado[i]];
      } catch (NullPointerException exception) {
          println("Erro ao calcular fitness");
      }
    }
  }
  
  public void somatorioOrdenado() {
    this.somatorioOrdenado = ordenaIndiceDecrescente(somatorio);
  }
  
   public void selecionarMelhoresRedes() { 
     for(int i = 0; i < this.melhoresPID.length; i++) bestWeights[i] = weightsAndBias[somatorioOrdenado[i]].copy();
   }
   
   public void crossOverNetwork() {
      for (int i = 0; i < populacao.length; i++) {
        int aleatorio = (int) random(0, this.numMelhores);
        int second = (int) random(0, this.numMelhores);
        if(second == aleatorio) second = (int) random(0, this.numMelhores);
        if(random(1.0) <= 0.2) weightsAndBias[i] = neuronMaster.join(bestWeights[aleatorio].copy(), weightsAndBias[i].copy(), mutacao, dP).copy();
        else weightsAndBias[i] = neuronMaster.join(bestWeights[aleatorio].copy(), bestWeights[second], mutacao, dP).copy();
      }
      weightsAndBias[populacao.length / 2] = bestWeights[0].copy();
    }
  
  public void selecionarMelhores() {
    for(int i = 0; i < melhoresPID.length; i++) melhoresPID[i] = populacao[somatorioOrdenado[i]].clone(); 
  }
  
  public void crossOver() {
    int i, j;
    int aleatorio; // Índice aleatório de 0 a melhoresPID.length - 1
    for(i = 0; i < populacao.length; i++) {
      aleatorio = (int) random(0, melhoresPID.length);
      for(j = 0; j < 4; j++) {
        if(random(1) <= mutacao) {
          //if(random(1) < 0.5) populacao[i][j] = ceil(random(populacao[i][j], amplitude));
          //else populacao[i][j] = ceil(random(0, amplitude));
          populacao[i][j] = ceil(random(0, amplitude));
        } else {
          if(random(1) < 0.5) populacao[i][j] = melhoresPID[aleatorio][j];
        }
      }
      //if(random(1) < 0.1) populacao[i] = melhoresPID[i/melhoresPID.length].clone();
    }
    //populacao[(int) random(populacao.length)] = melhoresPID[0].clone();
    //populacao[(int) random(populacao.length)] = melhoresPID[(int) random(melhoresPID.length)].clone();
  }
  
  private boolean loucura() {
    char i, j;
    for(i = 0; i < 10; i++) {
      for(j = 0; j < 4; j++) {
        if(melhoresPID[i][j] != 0) return true;
      }
    }
    return false;
  }
  
  private void psoVetValues() {
    for(int i = 0; i < tamPopulacao; i++) {
      for(int j = 0; j < neuronMaster.getVetValuesLength(); j++) weightsAndBias[i].mat[j][0] = pso.pop[i].pose[0].coord[j];
    }
  }
  
  public void novaGeracaoPSO() {
    networkTest();
    pso.iteration();
    psoVetValues();
    selecionarMelhoresRedes();
    geracaoAtual++;
  }
  
  public void novaGeracaoNeural() {
    networkTest();
    selecionarMelhoresRedes();
    crossOverNetwork();
    geracaoAtual++;
  }
  
  public void novaGeracao() {
    testarPopulacao();
    selecionarMelhores();
    crossOver();
    geracaoAtual++;
  }
  
  public void printFitnessBest() {
    //println("Fitness: " + nf(fitness[0]*100.0, 1, 2) + " %");
    println(fitness[0]);
  }
  
  public void printFitness() {
    println("Fitness: " + nf(fitness[0]*100.0, 1, 2) + " %");
  }
  
  public void printFitness(int i) {
    println("Fitness: " + nf(fitness[i]*100.0, 1, 2) + " %");
  }
  
  public void printFitness(int s1, int s2) {
    println("Fitness: " + nf(fitness[0]*100.0, s1, s2) + " %");
  }
  
  public void printFitness(int i, int s1, int s2) {
    println("Fitness: " + nf(fitness[i]*100.0, s1, s2) + " %");
  }
  
  public void printGeracoesPID() {
    println("Geracao: " + this.geracaoAtual + ", PID nº " + this.pidSelecionado);
  }
  
  public void printGeracoesRede() {
    println("Geracao: " + this.geracaoAtual + ", Rede nº " + this.pidSelecionado);
  }
  
  public void printPID(int[] values) {
    float[][] intervalo = robot.getIntervalo();
    println("Kp: " + map(values[0], 0, amplitude, intervalo[0][0], intervalo[0][1]));
    println("Ki: " + map(values[1], 0, amplitude, intervalo[1][0], intervalo[1][1]));
    println("Kd: " + map(values[2], 0, amplitude, intervalo[2][0], intervalo[2][1]));
    println("Vel: " + (int) map(values[3], 0, amplitude, intervalo[3][0], intervalo[3][1]));
    print("\n------------------------------\n\n");
  }
  
  public void printPID(int[] values, float[][] intervalo) {
    println("Kp: " + map(values[0], 0, amplitude, intervalo[0][0], intervalo[0][1]));
    println("Ki: " + map(values[1], 0, amplitude, intervalo[1][0], intervalo[1][1]));
    println("Kd: " + map(values[2], 0, amplitude, intervalo[2][0], intervalo[2][1]));
    println("Vel: " + (int) map(values[3], 0, amplitude, intervalo[3][0], intervalo[3][1]));
    print("\n------------------------------\n\n");
  }
  
  public void printVetor(float[] vet) {
    int i;
    print("{" + vet[0]);
    for(i = 1; i < vet.length; i++) print(", " + vet[i]);
    print("}\n");
  }
  
  public void printVetor(float[] vet, int s1, int s2) {
    int i;
    print("{" + nf(vet[0], s1, s2));
    for(i = 1; i < vet.length; i++) print(", " + nf(vet[i], s1, s2));
    print("}\n");
  }
  
  public void printVetor(int[] vet) {
    int i;
    print("{" + vet[0]);
    for(i = 1; i < vet.length; i++) print(", " + vet[i]);
    print("}\n");
  }
  
  public int[] ordenaIndiceCrescente(float[] vet) {
    float[] vetor = vet.clone();
    int[] ind = new int[vetor.length];
    int aux, i, j;
    float maior;
    for(i = 0; i < ind.length; i++) ind[i] = i;
    for(i = 0; i < vetor.length; i++) {
      for(j = 0; j < vetor.length; j++) {
        if(vetor[i] < vetor[j]) {
          maior = vetor[i];
          vetor[i] = vetor[j];
          vetor[j] = maior;
          aux = ind[i];
          ind[i] = ind[j];
          ind[j] = aux;
        }
      }
    }
    return ind;
  }
  
  public int[] ordenaIndiceDecrescente(float[] vet) {
    float[] vetor = vet.clone();
    int[] ind = new int[vetor.length];
    int aux, i, j;
    float maior;
    for(i = 0; i < ind.length; i++) ind[i] = i;
    for(i = 0; i < vetor.length; i++) {
      for(j = 0; j < vetor.length; j++) {
        if(vetor[i] < vetor[j]) {
          maior = vetor[i];
          vetor[i] = vetor[j];
          vetor[j] = maior;
          aux = ind[i];
          ind[i] = ind[j];
          ind[j] = aux;
        }
      }
    }
    for(i = 0; i < ind.length/2; i++) {
      aux = ind[i];
      ind[i] = ind[ind.length - i - 1];
      ind[ind.length - i - 1] = aux;
    }
    return ind;
  }
  
  public void setMutacao(float mutation) {
    if(mutation >= 0 && mutation <= 1.0) this.mutacao = mutation;
  }
  
  public void testeOrdena(int[] v1, float[] v2) {
    float[] vet = new float[v2.length];
    for(int i = 0; i < v2.length; i++) {
      vet[i] = v2[v1[i]];
    }
    printVetor(vet, 1, 2);
  }
  
  public Matrix[] getBestWeights() {
    return this.bestWeights;
  }
  
  public Matrix getBestWeights(int i) { 
    return this.bestWeights[i];
  }

  public Matrix[] getWeightsAndBias() { 
    return this.weightsAndBias; 
  }
  
  public Matrix getWeightsAndBias(int i) { 
    return this.weightsAndBias[i]; 
  }

  public float getFitness(int i) {
    return fitness[i];
  }
  
  public float getVelMedia(int i) {
    return velocidades[somatorioOrdenado[i]];
  }
  
  public float getDistPercorrida(int i) {
    return distPercorrida[somatorioOrdenado[i]];
  }
  
  public float getTempoGasto(int i) {
    return tempoGasto[somatorioOrdenado[i]];
  }
  
  public float getErro(int i) {
    return erro[somatorioOrdenado[i]];
  }
  
  public int getGeracaoAtual() {
    return this.geracaoAtual;
  }
  
  public float getMutacao() {
    return this.mutacao;
  }
  
  public int[] getPopulacao(int i) {
    return populacao[i].clone();
  }
  
  public int[][] getPopulacao() {
    return populacao.clone();
  }
  
  public int[] getMelhoresPID(int i) {
    this.pidSelecionado = i;
    return melhoresPID[i].clone();
  }
  
  public int[][] getMelhoresPID() {
    return melhoresPID.clone();
  }
  
  public int getTamanlhoPopulacao() {
    return this.tamPopulacao;
  }
  
  public String getPesos() {
    String pesos = nf(pesoVel, 1, 2) + " %\n" + nf(pesoDist, 1, 2) + " %\n" + nf(pesoEstabilidade, 1, 2);
    return pesos; 
  }
  
}
