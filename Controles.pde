public class Control {
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float ts = 0;
  float tf = 0;
  char  type = 'L';
  float vel = 0;
  float errorMin = 50000;
  float integral = 0;
  float lastError = 0;
  float last_derivative = 0;
  float rangeError = 11;
  String[] log_file = null;
  boolean save_data = false;
  
  Control() {
  }
  
  Control(float p, float i, float d) {
    this.Kp = p;
    this.Ki = i;
    this.Kd = d;
  }
  
  Control(float p, float i, float d, float v) {
    this.Kp = p;
    this.Ki = i;
    this.Kd = d;
    this.vel = v;
  }
  
  Control(float p, float i, float d, float v, char t) {
    this.Kp = p;
    this.Ki = i;
    this.Kd = d;
    this.vel = v;
    this.type = t;
  }
  
  public void setControl(float p, float i, float d) {
    this.Kp = p;
    this.Ki = i;
    this.Kd = d;
  }
  
  public void setControl(float p, float i, float d, float v) {
    this.Kp = p;
    this.Ki = i;
    this.Kd = d;
    this.vel = v;
  }
  
  public void setControl(float p, float i, float d, float v, char t) {
    this.Kp = p;
    this.Ki = i;
    this.Kd = d;
    this.vel = v;
    this.type = t;
  }
  
  public Control copy() {
    Control c = new Control(Kp, Ki, Kd, vel, type);
    c.tf = tf;
    c.ts = ts;
    c.errorMin = errorMin;
    c.rangeError = rangeError;
    return c;
  }
  
  public void saveLogFile() {
    String[] txt = new String[1];
    txt[0] = this.log_file[0];
    println("Salvando log_file");
    saveStrings("controle/vel_linear.txt", txt);
    txt[0] = this.log_file[1];
    saveStrings("controle/vel_estrategia.txt", txt);
    txt[0] = this.log_file[2];
    saveStrings("controle/dist_percorrida.txt", txt);
    txt[0] = this.log_file[3];
    saveStrings("controle/tempo.txt", txt);
    this.log_file = null;
    println("Salvo");
  }
  
  public void saveDegrau() {
    String[] txt = new String[1];
    txt[0] = this.log_file[0];
    println("Salvando degrau_file");
    saveStrings("degrau/vel_esq.txt", txt);
    txt[0] = this.log_file[1];
    saveStrings("degrau/vel_dir.txt", txt);
    txt[0] = this.log_file[2];
    saveStrings("degrau/dist_percorrida.txt", txt);
    txt[0] = this.log_file[3];
    saveStrings("degrau/tempo.txt", txt);
    this.log_file = null;
    println("Salvo");
  }
}
