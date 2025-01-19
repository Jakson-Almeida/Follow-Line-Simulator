public class PSO { // Particle Swarm Optimization
  private boolean minimize = false; // se é para encontrar o mínimo ou máximo geral
  public  int tam = 10;         // tamanho da população
  private int N = 1;           // dimensão de indivíduos
  public  int n_coord = 1;     // para cada indivíduo tem sua dimensão em coordenadas espaciais
  public  Individual[] pop;    // população
  private int bestPoseInd = 0; // índice para o indivíduo com melhor pose, melhor fitness
  private int iterate = 0;    // número de iterações
  
  PSO() {
    startPop();
  }
  
  public void findFitness() {
    for(int i = 0; i < tam; i++) {
      //pop[i].fitness =
      if(minimize) {
        if(pop[i].fitness < pop[i].best_fitness) {
          pop[i].best_fitness = pop[i].fitness;
          pop[i].bestPoseIndividual = pop[i].copyPose();
        }
      } else {
        if(pop[i].fitness > pop[i].best_fitness) {
          pop[i].best_fitness = pop[i].fitness;
          pop[i].bestPoseIndividual = pop[i].copyPose();
        }
      }
    }
  }
  
  public void findBestPose() { // Selecionar maior fitness
    bestPoseInd = 0;
    pop[0].isTheBest = false; // UPDATE IN 22/08/2020
    for(int i = 1; i < tam; i++) {
      pop[i].isTheBest = false;
      if(minimize) {
        if(pop[i].fitness < pop[bestPoseInd].fitness) {
          bestPoseInd = i;
        }
      } else {
        if(pop[i].fitness > pop[bestPoseInd].fitness) {
          bestPoseInd = i;
        }
      }
    }
    pop[bestPoseInd].isTheBest = true;
  }
  
  public void updatePose() {
    for(int i = 0; i < tam; i++) 
    //pop[i].updatePose(pop[bestPoseInd].getPose());
    pop[i].updatePose(pop[bestPoseInd].getPose(), pop[bestPoseInd].fitness);
  }
  
  public void iteration() {
    findFitness();
    findBestPose();
    updatePose();
    iterate++;
    //println("Iteração: " + iterate);
  }
  
  public void setSize(int t) {
    if(t < 1) return;
    tam = t;
    startPop();
  }
  
  public void setDimension(int n) {
    if(n < 1) return;
    N = n;
    startPop();
  }
  
  public void setCoord(int c) {
    if(c < 1) return;
    n_coord = c;
    startPop();
  }
  
  public void startPop() {
    pop = new Individual[tam];
    for(int i = 0; i < tam; i++) {
      pop[i] = new Individual(N, n_coord);
      if(minimize) {
        pop[i].fitness      *= -1;
        pop[i].best_fitness *= -1;
      }
      pop[i].randomPose();
      pop[i].minimize = this.minimize;
    }
  }
}

public class Individual {
  boolean minimize = true;
  boolean isTheBest = false;
  int N = 1;
  int num = 1;
  float fitness = -9999999;
  float best_fitness = -9999999; // Melhor valor para fitness
  float k_1 = 1;
  float k_2 = 1;
  float w = 0.3; // Constante para o somatório de velocidade
  float dT = 0.4; // Em segundos
  float k_l = 5;
  float k_d = 0.1;
  float k_f = 110;
  Vector[] pose;
  Vector[] speed;
  Vector[] bestPoseIndividual;
  
  Individual() {
    this.pose = new Vector[N];
    this.pose[0] = new Vector(num);
    this.speed = new Vector[N];
    this.speed[0] = new Vector(num);
    this.bestPoseIndividual = copyPose();
  }
  
  Individual(int _N, int number) {
    if(_N > 0) this.N = _N;
    if(number > 0) this.num = number;
    this.pose = new Vector[N];
    for(int i = 0; i < N; i++) this.pose[i] = new Vector(num);
    this.speed = new Vector[N];
    for(int i = 0; i < N; i++) this.speed[i] = new Vector(num);
    this.bestPoseIndividual = copyPose();
  }
  
  public float eucladianDistance(float[] v1, float[] v2) {
    float sum = 0;
    if(v1.length != v2.length) return 9999999;
    for(int i = 0; i < v1.length; i++) sum += (v1[i]-v2[i])*(v1[i]-v2[i]);
    return sqrt(sum);
  }
  
  public void updatePose(Vector[] ps) {
    float phi_1;
    float phi_2;
    for(int i = 0; i < pose.length; i++) {
      phi_1 = random(1.0)*k_1;
      phi_2 = random(1.0)*k_2;
      for(int j = 0; j < num; j++) {
        speed[i].coord[j] = w*speed[i].coord[j] + (ps[i].coord[j] - pose[i].coord[j])*phi_1 + (bestPoseIndividual[i].coord[j] - pose[i].coord[j])*phi_2;
        pose[i].coord[j] = pose[i].coord[j] + speed[i].coord[j]*dT;
        //pose[i].coord[j] = constrain(pose[i].coord[j], pose[i].n_min, pose[i].n_max);
      }
    }
  }
  
  public void updatePose(Vector[] ps, float fit) {
    float phi_1;
    float phi_2;
    for(int i = 0; i < pose.length; i++) {
      phi_1 = random(1.0)*k_1;
      phi_2 = random(1.0)*k_2;
      for(int j = 0; j < num; j++) {
        speed[i].coord[j] = w*speed[i].coord[j] + (ps[i].coord[j] - pose[i].coord[j])*phi_1 + (bestPoseIndividual[i].coord[j] - pose[i].coord[j])*phi_2;
        pose[i].coord[j] = pose[i].coord[j] + speed[i].coord[j]*dT;
        //pose[i].coord[j] = constrain(pose[i].coord[j], pose[i].n_min, pose[i].n_max);
      }
      //if(fitness < 0.01) println("estranho " + eucladianDistance(pose[i].coord, ps[i].coord) + " v1: " + pose[i].coord[0] + " v2: " + ps[i].coord[0]);
      if(random(1.0) < 0.1) 
      if((minimize) ? (fit > -k_f) : (fit < k_f)) 
      if(!isTheBest && eucladianDistance(pose[i].coord, ps[i].coord) < k_l) {
        for(int j = 0; j < num; j++) pose[i].coord[j] += random(-k_d, k_d);
      }
    }
  }
  
  public void randomPose() {
    for(int i = 0; i < N; i++) {
      pose[i].randomValues();
    }
  }
  
  public Vector[] getPose() {
    return pose;
  }
  
  public Vector[] copyPose() {
    Vector[] send = new Vector[pose.length];
    for(int i = 0; i < pose.length; i++) send[i] = pose[i].get();
    return send;
  }
}

public class Vector {
  int num = 1;
  float n_max = 2; // max > min
  float n_min = 0;
  float[] coord;
  
  Vector(int num) { // Número de variáveis
    if(num > 0) this.num = num;
    this.coord = new float[this.num];
  }
  
  public void randomValues() {
    for(int i = 0; i < num; i++) coord[i] = random(n_min, n_max);
  }
  
  public void randomValues(float min, float max) {
    for(int i = 0; i < num; i++) coord[i] = random(min, max);
  }
  
  public Vector get() {
    Vector send = new Vector(num);
    for(int i = 0; i < coord.length; i++) send.coord[i] = this.coord[i];
    return send;
  }
}
