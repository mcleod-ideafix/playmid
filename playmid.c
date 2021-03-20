/*
 * PLAYMID. ESXDOS command to play MIDI files in a ZX Spectrum 128K computer
 * Copyright (C) 2019 Miguel Angel Rodriguez Jodar
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
Compilar con:
sdcc -mz80 --reserve-regs-iy --opt-code-size --max-allocs-per-node 10000 ^
--nostdlib --nostdinc --no-std-crt0 --code-loc 0x2000 --data-loc 0x2b00 playmid.c z80.lib -L "C:\Program Files\SDCC\lib\z80"
makebin -s 65535 -p playmid.ihx playmid.bin
dd if=playmid.bin of=PLAYMID bs=1 skip=8192

OJO con --data-loc. Ahora mismo hay un margen de unos 180 bytes entre el final del códdigo (sin librerías) y el valor
que se indica en --data-loc. Si el código de este programa crece, habría que mover --data-loc adecuadamente para que no se
solapen. Ojala encuentre una forma de evitar esto y que los datos, sencillamente, se pongan al final de todo el código, o
entre mi código y las librerías, sin tener que especificar nada.

*/

typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;

__sfr __at (0xfe) ULA;
__sfr __at (0xff) ATTR;

__sfr __banked __at (0xf7fe) SEMIFILA1;
__sfr __banked __at (0xeffe) SEMIFILA2;
__sfr __banked __at (0xfbfe) SEMIFILA3;
__sfr __banked __at (0xdffe) SEMIFILA4;
__sfr __banked __at (0xfdfe) SEMIFILA5;
__sfr __banked __at (0xbffe) SEMIFILA6;
__sfr __banked __at (0xfefe) SEMIFILA7;
__sfr __banked __at (0x7ffe) SEMIFILA8;

__sfr __banked __at (0xfffd) AYREGSELECT;
__sfr __banked __at (0xbffd) AYREGWRITE;

__sfr __banked __at (0xfc3b) ZXUNOADDR;
__sfr __banked __at (0xfd3b) ZXUNODATA;

#define ATTRP 23693
#define ATTRT 23695
#define BORDR 23624
#define LASTK 23560

#define WAIT_VRETRACE __asm halt __endasm
#define WAIT_HRETRACE while(ATTR!=0xff)
#define SETCOLOR(x) *(BYTE *)(ATTRP)=(x)
#define LASTKEY *(BYTE *)(LASTK)
#define ATTRPERMANENT *((BYTE *)(ATTRP))
#define ATTRTEMPORARY *((BYTE *)(ATTRT))
#define BORDERCOLOR *((BYTE *)(BORDR))

#define MAKEWORD(d,h,l) { ((BYTE *)&(d))[0] = (l) ; ((BYTE *)&(d))[1] = (h); }

/* Some ESXDOS system calls */
#define HOOK_BASE   128
#define MISC_BASE   (HOOK_BASE+8)
#define FSYS_BASE   (MISC_BASE+16)
#define M_GETSETDRV (MISC_BASE+1)
#define F_OPEN      (FSYS_BASE+2)
#define F_CLOSE     (FSYS_BASE+3)
#define F_READ      (FSYS_BASE+5)
#define F_WRITE     (FSYS_BASE+6)
#define F_SEEK      (FSYS_BASE+7)
#define F_GETPOS    (FSYS_BASE+8)

#define FMODE_READ	     0x1 // Read access
#define FMODE_WRITE      0x2 // Write access
#define FMODE_OPEN_EX    0x0 // Open if exists, else error
#define FMODE_OPEN_AL    0x8 // Open if exists, if not create
#define FMODE_CREATE_NEW 0x4 // Create if not exists, if exists error
#define FMODE_CREATE_AL  0xc // Create if not exists, else open and truncate

#define SEEK_START       0
#define SEEK_CUR         1
#define SEEK_BKCUR       2

BYTE errno;

// Esta precisión la he elegido suponiendo que ppq nunca será mayor en la práctica de 2048, para que no 
// desborde en 32 bits al calcular el valor de ticks_per_int en un evento FF 03 58
#define PRECISION 64

// variables globales en lugar de locales para agilizar su lectura, y no depender de direccionamiento indexado
// que engordaría y enlentecería (más aún) el programa

BYTE formato;  // formato del fichero MIDI: 0, 1 o 2. En realidad esta variable es un poco superflua.
BYTE i, c;     // contadores de bucle, etc.
BYTE status;   // byte de estado del evento MIDI
BYTE param1, param2;  // bytes de parametros opcionales para el estado
BYTE running_status;  // a 1 si el evento actual no tiene estado.
DWORD ppq, delta, delta_ints, ticks_per_int, tick;  // variables que se usan para calcular el tempo de la melodía
__at(0x3000) BYTE buffer[1024];  // Buffer de 1KB para guardar eventos MIDI. No moverlo de aqui sin tocar SendMIDI
WORD pos, ppos, leido, lbytes;   // variables para movernos por el buffer. ppos se usa para saber si hemos pasado de una mitad a otra del buffer
DWORD us_per_quarter;    // ultimo tempo leido con el metaevento Set Tempo

BYTE main (char *p);
void usage (void);
BYTE commandlinemode (char *p);

void __sdcc_enter_ix (void) __naked;

void puts (BYTE *);
void u16tohex (WORD n, char *s);
void u8tohex (BYTE n, char *s);
void print8bhex (BYTE n);
void print16bhex (WORD n);

BYTE open (char *filename, BYTE mode);
void close (BYTE handle);
WORD read (BYTE handle, BYTE *buffer, WORD nbytes);

/* --------------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------------- */
void getfilename (char *p, char *fname);
void playmidi (BYTE f);
int cmp4b (BYTE *a, BYTE *b);
void SendMIDI (BYTE *ev, BYTE lev);
void SendMIDIByte (void) __naked;

// Rutina inicial. Debe ser el primer código que se encuentre en el fichero. Esta inicialización
// está pensada para ser usada con ficheros .command de ESXDOS.
void init (void) __naked
{
     __asm
     xor a
     ld (#_errno),a
     push hl
     call _main
     inc sp
     inc sp
     ld iy,#23610  ;algunas rutinas de la libreria Z80 tocan IY :(
     ld a,l
     or a
     ret z
     scf
     ret
     __endasm;
}

// Programa principal. Toma como argumento un puntero al comienzo de la linea de comandos. Si es NULL, no hay linea de comandos
BYTE main (char *p)
{
  BYTE res, comando;

  // Dejamos el puerto MIDI inactivo
  AYREGSELECT = 0x0e;
  AYREGWRITE = 0xfe;

  // GM MIDI Reset
  WAIT_VRETRACE;
  comando = 0xFF;   // envío el comando FF para resetear el MIDI
  SendMIDI (&comando, 1);

  // Si no hay fichero para abrir, mostrar ayuda
  if (!p)
  {
     usage();
     return 0;
  }
  else
      res = commandlinemode(p);

  // GM MIDI Reset
  WAIT_VRETRACE;
  comando = 0xFF;   // envío el comando FF para resetear el MIDI
  SendMIDI (&comando, 1);

  return res;
}

// Abre el fichero. Si no existe, retorna con error. Si existe, pasa su handle a la rutina principal.
BYTE commandlinemode (char *p)
{
    char fname[32];
    BYTE handle;

    getfilename (p, fname);
    handle = open (fname, FMODE_READ);
    if (handle==0xff)
       return errno;

    playmidi (handle);

    close (handle);
    return 0;
}

// Muestra ayuda de uso del comando
void usage (void)
{
        // 01234567890123456789012345678901
    puts (" PLAYMID file.mid\xd\xd"
          "Plays a MIDI format 0 file thru\xd"
          "MIDI OUT connector.\xd");
}

////////////////////////////////////////////////////////////////////////////////

// Rutina principal de reproducción MIDI
void playmidi (BYTE f)
{
    // Leemos los primeros 512 bytes del fichero, en donde se encuentra la cabecera MIDI (14 bytes)
    read (f, buffer, 512);

    // Comprobamos que realmente es una cabecera MIDI, y si no, retornamos con error
    if (cmp4b (buffer, "MThd") == 0)
    {
        puts ("MThd chunk expected\xd");
        return;
    }

    // Leemos el formato del fichero. Sólo aceptamos formato 0
    formato = buffer[9];
    if (formato != 0)
    {
        puts ("Only format 0 MIDI files. Sorry\xd");
        return;
    }

    // Leemos el PPQ (partes por quarter, o el numero de ticks del reloj de MIDI que dura una negra
    ppq = buffer[12]<<8 | buffer[13];

    //ticks_per_quarter = <PPQ from the header>
    //µs_per_quarter = <Tempo in latest Set Tempo event>
    //µs_per_tick = µs_per_quarter / ticks_per_quarter
    //seconds_per_tick = µs_per_tick / 1.000.000
    //seconds = ticks * seconds_per_tick

    // calculamos el numero de ticks MIDI que hay en una interrupción del Spectrum (20ms)
    // Este cálculo es distinto dependiendo del bit 7 del byte 12 de la cabecera
    if (buffer[12]&0x80)
    {
        buffer[12] &= 0x7F;
        ticks_per_int = PRECISION * buffer[12] * buffer[13] * 20;
    }
    else  // habitualmente los MIDs lo calculan de esta otra forma, es decir, habitualmente el bit 7 del byte 12 es 0.
    {
        ticks_per_int = PRECISION * 20 * ppq / 500;
    }

    // Comprobamos que realmente es una pista MIDI, y si no es así, retornamos con error.
    if (cmp4b (buffer+14, "MTrk") == 0)
    {
        puts ("MTrk chunk expected\xd");
        return;
    }

    // A partir de aquí, lo que hay en el MIDI son eventos.
    pos = 22;   // nos situamos al comienzo del primer evento de la pista
    ppos = 512; // forzamos a que se vaya leyendo la segunda mitad del buffer
    tick = 0;
    running_status = 0;

    while(1)
    {
        // Si pulsamos SPACE, salir
        if ((SEMIFILA8 & 0x1) == 0)
          return;

        // Si al recorrer el buffer con pos, pasamos de una mitad a otra del buffer, recargamos de fichero la mitad que acabamos de usar
        if ((pos&0x200) != (ppos&0x200))
        {
            if (pos&0x200)
                read (f, buffer, 512);
            else
                read (f, buffer+512, 512);
            ppos = pos;
        }

        // Leemos y calculamos el delta (numero de ticks del reloj MIDI que han de pasar desde ahora) del evento actual
        delta = 0;
        do
        {
            c = buffer[pos];
            pos = (pos + 1) & 0x3FF;
            delta = (delta<<7) | (c &0x7F);
        }
        while(c & 0x80);

        // Si delta especifica un tiempo superior a 0...
        if (delta)
        {
           // ajustamos delta segun la precisión (aritmética de punto fijo)
           delta *= PRECISION;
           // cada interrupción dura ticks_per_int ticks de reloj MIDI. Con este bucle esperamos un
           // numero de interrupciones que sea igual o algo mayor al que dicte el numero de ticks en delta.
           while (tick < delta)
           {
               if ((SEMIFILA8 & 0x1) == 0)
                  return;
               WAIT_VRETRACE;
               tick += ticks_per_int;
           }
           tick -= delta;  // ajustamos tick por si hemos esperado más de la cuenta (para compensar)
        }
        // si lo que hay tras el delta es un byte de estado (con bit 7=1), entonces cancelamos running status
        if (buffer[pos] & 0x80)
        {
            running_status = 0;
            status = buffer[pos];
        }
        else // per si no, entonces status no se actualiza y señalamos que hay running status
            running_status = 1;

        // ahora, según el valor de status, estamos ante un evento u otro. Los procesamos.
        
        // EVENTOS F0 y F7. Se usan para enviar SYSEX y datos arbitrarios al secuenciador. Los pasmos tal cual.
        if (status == 0xF0 || status == 0xF7)
        {
          pos = (pos + 1) & 0x3FF;
          lbytes = 0;
          do
          {
              c = buffer[pos];
              pos = (pos + 1) & 0x3FF;
              lbytes = (lbytes<<7) | (c & 0x7F);
          }
          while (c & 0x80);

          if (status == 0xF0)
          {
            SendMIDI (&status, 1);
            SendMIDI (buffer+pos, lbytes);
          }
          else
          {
            SendMIDI (buffer+pos, lbytes);
          }
          pos = (pos + lbytes) & 0x3FF;
          continue;
        }

        // EVENTOS 8n, 9n, An, Bm, Cn, Dn, En y Fn. Si es alguno de estos, se envia el evento MIDI asociado.
        // En estos eventos se ha de tener en cuenta el running status para enviar 1, 2 o 3 bytes según sea el caso.
        c = (status>>4) & 0xF;
        if (c == 0x8 || c == 0x9 || c == 0xA || c == 0xB || c == 0xE)
        {
            if (running_status == 0)
            {
                SendMIDI (buffer+pos, 3);
                pos = (pos + 3) & 0x3FF;
            }
            else
            {
                SendMIDI (buffer+pos, 2);
                pos = (pos + 2) & 0x3FF;
            }
            continue;
        }
        if (c == 0xC || c == 0xD)
        {
            if (running_status == 0)
            {
                SendMIDI (buffer+pos, 2);
                pos = (pos + 2) & 0x3FF;
            }
            else
            {
                SendMIDI (buffer+pos, 1);
                pos = (pos + 1) & 0x3FF;
            }
            continue;
        }

        // EVENTOS que necesitan un primer parámetro
        // estos eventos serán ignorados
        pos = (pos + 1) & 0x3FF;
        param1 = buffer[pos];
        if (status == 0xFF && ((param1 >= 0x01 && param1 <= 0x07) || param1 == 0x7F))
        {
            pos = (pos + 1) & 0x3FF;
            lbytes = 0;
            do
            {
                c = buffer[pos];
                pos = (pos + 1) & 0x3FF;
                lbytes = (lbytes<<7) | (c & 0x7F);
            }
            while (c & 0x80);

            pos = (pos + lbytes) & 0x3FF;
            continue;
        }

        // EVENTOS que necesitan un segundo parámetro
        pos = (pos + 1) & 0x3FF;
        param2 = buffer[pos];
        if (status == 0xFF)
        {
            if (param1 == 0x2F && param2 == 0x00)  // Meta evento de FIN DE PISTA. Terminar con el bucle de reproducción
                break;
            if (param1 == 0x51 && param2 == 0x03)  // Meta evento Set Tempo.
            {
                us_per_quarter = 0;
                for (i=0;i<3;i++)                  // Leemos los 24 bits que forman el nuevo valor de us_per_quarter
                {
                    pos = (pos + 1) & 0x3FF;
                    us_per_quarter = (us_per_quarter<<8) | buffer[pos];
                }
                pos = (pos + 1) & 0x3FF;
                ticks_per_int = PRECISION * 20000L * ppq / us_per_quarter;  // y calculamos el nuevo valor de ticks_per_int
            }
            // El resto de meta eventos se ignora. No sé si hago bien en ignorarlos todos, pero de momento, parece que ningún MIDI se ha quejado
            if (param1 == 0x00 && param2 == 0x02)
            {
                pos = (pos + 3) & 0x3FF;
            }
            if (param1 == 0x20 && param2 == 0x01)
            {
                pos = (pos + 2) & 0x3FF;
            }
            if (param1 == 0x21 && param2 == 0x01)
            {
                pos = (pos + 2) & 0x3FF;
            }
            if (param1 == 0x54 && param2 == 0x05)
            {
                pos = (pos + 6) & 0x3FF;
            }
            if (param1 == 0x58 && param2 == 0x04)
            {
                pos = (pos + 5) & 0x3FF;
            }
            if (param1 == 0x59 && param2 == 0x02)
            {
                pos = (pos + 3) & 0x3FF;
            }
            continue;
        }
    }
}

// Rutina para comparar rapidamente dos bloques de memoria de 4 bytes
int cmp4b (BYTE *a, BYTE *b)
{
    if (a[0] != b[0] || a[1] != b[1] || a[2] != b[2] || a[3] != b[3])
        return 0;
    else
        return 1;
}

// Estos pragmas son para que no se queje el compilador por argumentos aparentemente no usados
#pragma disable_warning 85
#pragma disable_warning 59

// Envia un bloque de memoria a la salida MIDI del Spectrum
void SendMIDI (BYTE *ev, BYTE lev)
{
  BYTE d;

  // Si estamos en el ZXUNO, esta operación debe hacerse a la velocidad estándar (3.5 MHz)
  ZXUNOADDR = 0xb;
  d = ZXUNODATA;
  ZXUNODATA = d & 0x3F;
  __asm
  push bc
  push de
  ld l,4(ix)
  ld h,5(ix)
  ld b,6(ix)

buc_send_midi:
  ld a,(hl)
  push bc
  push hl
  di
  call _SendMIDIByte   ;llamada a la rutina para enviar un byte por MIDI
  ei
  pop hl
  pop bc
  inc hl
  ld a,h
  and #0xF3  ;esto asume que el buffer esta en 0x3000 - 0x33FF
  ld h,a
  djnz buc_send_midi

  pop de
  pop bc
  __endasm;

  // ZXUNO: restauramos la velocidad nominal que hubiera
  ZXUNODATA = d;
}

// Esta es una copia total de la rutina de la ROM 1 del 128K para enviar un byte MIDI.
// El fuente proviene del desensamble que está en la página de Paul Farrow. Si hubiera problemas con este cachito de código
// lo reescribiré.
void SendMIDIByte (void) __naked
{
  __asm
L11A3:  LD   L,A          ; Store the byte to send.

        LD   BC,#0xFFFD     ;
        LD   A,#0x0E        ;
        OUT  (C),A        ; Select register 14 - I/O port.

        LD   BC,#0xBFFD     ;
        LD   A,#0xFA        ; Set RS232 'RXD' transmit line to 0. (Keep KEYPAD 'CTS' output line low to prevent the keypad resetting)
        OUT  (C),A        ; Send out the START bit.

        LD   E,#0x03        ; (7) Introduce delays such that the next bit is output 113 T-states from now.

L11B4:  DEC  E            ; (4)
        JR   NZ,L11B4     ; (12/7)

        NOP               ; (4)
        NOP               ; (4)
        NOP               ; (4)
        NOP               ; (4)

        LD   A,L          ; (4) Retrieve the byte to send.

        LD   D,#0x08        ; (7) There are 8 bits to send.

L11BE:  RRA               ; (4) Rotate the next bit to send into the carry.
        LD   L,A          ; (4) Store the remaining bits.
        JP   NC,L11C9     ; (10) Jump if it is a 0 bit.

        LD   A,#0xFE        ; (7) Set RS232 'RXD' transmit line to 1. (Keep KEYPAD 'CTS' output line low to prevent the keypad resetting)
        OUT  (C),A        ; (11)
        JR   L11CF        ; (12) Jump forward to process the next bit.

L11C9:  LD   A,#0xFA        ; (7) Set RS232 'RXD' transmit line to 0. (Keep KEYPAD 'CTS' output line low to prevent the keypad resetting)
        OUT  (C),A        ; (11)
        JR   L11CF        ; (12) Jump forward to process the next bit.

L11CF:  LD   E,#0x02        ; (7) Introduce delays such that the next data bit is output 113 T-states from now.

L11D1:  DEC  E            ; (4)
        JR   NZ,L11D1     ; (12/7)

        NOP               ; (4)
        ADD  A,#0x00        ; (7)

        LD   A,L          ; (4) Retrieve the remaining bits to send.
        DEC  D            ; (4) Decrement the bit counter.
        JR   NZ,L11BE     ; (12/7) Jump back if there are further bits to send.

        NOP               ; (4) Introduce delays such that the stop bit is output 113 T-states from now.
        NOP               ; (4)
        ADD  A,#0x00        ; (7)
        NOP               ; (4)
        NOP               ; (4)

        LD   A,#0xFE        ; (7) Set RS232 'RXD' transmit line to 1. (Keep KEYPAD 'CTS' output line low to prevent the keypad resetting)
        OUT  (C),A        ; (11) Send out the STOP bit.

        LD   E,#0x06        ; (7) Delay for 101 T-states (28.5us).

L11E7:  DEC  E            ; (4)
        JR   NZ,L11E7     ; (12/7)
        RET
__endasm;
}

////////////////////////////////////////////////////////////////////////////////

// Copia desde una posición de memoria, los bytes que forman el nombre de un fichero hasta encontrar un espacio,
// un retorno de linea, o el simbolo de los dos puntos ":" para indicar fin de una sentencia. Se usa en ESXDOS
void getfilename (char *p, char *fname)
{
    while (*p!=':' && *p!=0xd && *p!=' ')
          *fname++ = *p++;
    *fname = '\0';
    return;
}


/* --------------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------------- */
// RUTINAS ACCESORIAS QUE SIEMPRE TENGO EN MIS PROGRAMAS ESXDOS PARA LO QUE HAGA FALTA.
// Creo que el compilador no las incluye en el binario final si no se usan.

#pragma disable_warning 85
#pragma disable_warning 59

void puts (BYTE *str)
{
  __asm
  push bc
  push de
  ld a,(#ATTRT)
  push af
  ld a,(#ATTRP)
  ld (#ATTRT),a
  ld l,4(ix)
  ld h,5(ix)
buc_print:
  ld a,(hl)
  or a
  jp z,fin_print
  cp #4
  jr nz,no_attr
  inc hl
  ld a,(hl)
  ld (#ATTRT),a
  inc hl
  jr buc_print
no_attr:
  rst #16
  inc hl
  jp buc_print

fin_print:
  pop af
  ld (#ATTRT),a
  pop de
  pop bc
  __endasm;
}

void u16tohex (WORD n, char *s)
{
  u8tohex((n>>8)&0xFF,s);
  u8tohex(n&0xFF,s+2);
}

void u8tohex (BYTE n, char *s)
{
  BYTE i=1;
  BYTE resto;

  resto=n&0xF;
  s[1]=(resto>9)?resto+55:resto+48;
  resto=n>>4;
  s[0]=(resto>9)?resto+55:resto+48;
  s[2]='\0';
}

void print8bhex (BYTE n)
{
    char s[3];

    u8tohex(n,s);
    puts(s);
}

void print16bhex (WORD n)
{
    char s[5];

    u16tohex(n,s);
    puts(s);
}

// Codigo necesario como prologo de cualquier función C. No se aplica a las __naked.
// Como no estoy incluyendo las librerias estándar ni el crt0 estándar, he de ponerla aqui
void __sdcc_enter_ix (void) __naked
{
    __asm
    pop	hl	; return address
    push ix	; save frame pointer
    ld ix,#0
    add	ix,sp	; set ix to the stack frame
    jp (hl)	; and return
    __endasm;
}

///////////////////////////////////////////////////////////////////////////////////////////
// RUTINAS ESXDOS (sólo las que necesito en este programa, por lo que no están ni seek ni write
///////////////////////////////////////////////////////////////////////////////////////////

BYTE open (char *filename, BYTE mode)
{
    __asm
    push bc
    push de
    xor a
    rst #8
    .db #M_GETSETDRV   ;Default drive in A
    ld l,4(ix)  ;Filename pointer
    ld h,5(ix)  ;in HL
    ld b,6(ix)  ;Open mode in B
    rst #8
    .db #F_OPEN
    jr nc,open_ok
    ld (#_errno),a
    ld a,#0xff
open_ok:
    ld l,a
    pop de
    pop bc
    __endasm;
}

void close (BYTE handle)
{
    __asm
    push bc
    push de
    ld a,4(ix)  ;Handle
    rst #8
    .db #F_CLOSE
    pop de
    pop bc
    __endasm;
}

WORD read (BYTE handle, BYTE *buffer, WORD nbytes)
{
    __asm
    push bc
    push de
    ld a,4(ix)  ;File handle in A
    ld l,5(ix)  ;Buffer address
    ld h,6(ix)  ;in HL
    ld c,7(ix)
    ld b,8(ix)  ;Buffer length in BC
    rst #8
    .db #F_READ
    jr nc,read_ok
    ld (#_errno),a
    ld bc,#65535
read_ok:
    ld h,b
    ld l,c
    pop de
    pop bc
    __endasm;
}
