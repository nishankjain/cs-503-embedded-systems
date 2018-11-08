volatile long enc_count = 0;
 
void setup() {
    Serial.begin(9600);
    attachInterrupt(0,encoder_isr,CHANGE);
    attachInterrupt(1,encoder_isr,CHANGE);
}

void encoder_isr() {
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;
    enc_val = enc_val | ((PIND & 0b1100) >> 2);
    
    enc_count = enc_count + lookup_table[enc_val & 0b1111];
    Serial.print(enc_count);
    Serial.print(", ");
    
}

void loop() {
}
