public class Button {
  private boolean sobre = false;
  private int   deslocamento = 0;
  private float variacao     = 500;
  private float x;
  private float y;
  private float comprimento;
  private float largura;
  private float raio;
  private color cor = color(127);
  
  Button() {
    this.sobre = false;
    this.x = 0;
    this.y = 0;
    this.comprimento = width;
    this.largura = height;
  }
  
  Button(float x, float y) {
    this.x = x;
    this.y = y;
  }
  
  Button(float x, float y, float c, float l) {
    this.x = x;
    this.y = y;
    this.comprimento = c;
    this.largura = l;
    rectButton();
  }
  
  Button(float x, float y, float r) {
    this.x = x;
    this.y = y;
    this.raio = r;
    circleButton();
  }
  
  public void rectButton(float x, float y, float c, float l) {
    if((mouseX >= x && mouseX <= (mouseX + c)) && (mouseY >= y && mouseY <= (y + l))) sobre = true;
    else sobre = false;
  }
  
  public void circleButton(float x, float y, float r) {
    if(sqrt(pow(mouseX - x, 2) + pow(mouseY - y, 2)) <= r) sobre = true;
    else sobre = false;
  }
  
  public void rectButton() {
    if((mouseX >= this.x && mouseX <= (this.x + this.comprimento)) && (mouseY >= this.y && mouseY <= (this.y + this.largura))) sobre = true;
    else sobre = false;
  }
  
  public void drawRectButton() {
    if((mouseX >= this.x && mouseX <= (this.x + this.comprimento)) && (mouseY >= this.y && mouseY <= (this.y + this.largura))) sobre = true;
    else sobre = false;
    noStroke();
    fill(cor);
    ellipse(this.x + this.comprimento/2.0, this.y + this.largura/2.0, this.comprimento, this.largura*0.2);
    if(mousePressed && sobre) {
      this.deslocamento = mouseX - ceil(this.x);
      this.variacao = 1000.0*this.deslocamento/this.comprimento;
      rect(mouseX - this.comprimento/30.0, y, this.comprimento/15.0, this.largura);
    } else rect(this.comprimento*variacao/1000.0 + this.x - this.comprimento/30.0, y, this.comprimento/15.0, this.largura);
  }
  
  public void circleButton() {
    if(sqrt(pow(mouseX - this.x, 2) + pow(mouseY - this.y, 2)) <= this.raio) sobre = true;
    else sobre = false;
  }
  
  public boolean circleIsPressed() {
    circleButton();
    return this.sobre;
  }
  
  public boolean rectIsPressed() {
    rectButton();
    return this.sobre;
  }
  
  public boolean isItOn() {
    return sobre;
  }
  
  public void setX(float x) {
    this.x = x;
  }
  
  public void setY(float y) {
    this.y = y;
  }
  
  public void setComprimento(float c) {
    this.comprimento = c;
  }
  
  public void setVariacao(int v) {
    if(v > 0 && v <= 1000) this.variacao = v;
  }
  
  public void setCor(color c) {
    this.cor = c;
  }
  
  public void setLargura(float l) {
    this.largura = l;
  }
  
  public color getCor() {
    return this.cor;
  }
  
  public float getVariacao() {
    return this.variacao;
  }
  
  public float getX() {
    return this.x;
  }
  
  public float getY() {
    return this.y;
  }
  
  public float getComprimento() {
    return this.comprimento;
  }
  
  public float getLargura() {
    return this.largura;
  }
}
