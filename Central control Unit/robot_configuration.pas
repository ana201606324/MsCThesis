unit Robot_Configuration;

//comunica com os robots e com o sistema de localizaçao

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, lNetComponents, Forms, Controls, Graphics,
  Dialogs, StdCtrls, ExtCtrls, IniPropStorage, SdpoSerial, LCLIntf, LCLType,
  ComCtrls, ActnList, channels, math, lNet, Utils;

type
   Matrix2D = array of array of Double;
   DoubleArray = array of Double;
   IntArray = array of integer;

  { TFRobot_Configuration }

  TFRobot_Configuration = class(TForm)
    BSendR1: TButton;
    BSendR2: TButton;
    BSendR3: TButton;
    Button2: TButton;
    EditBatVoltage: TEdit;
    EditBatVoltage1: TEdit;
    EditBatVoltage2: TEdit;
    EditM0Speed: TEdit;
    EditM0Speed1: TEdit;
    EditM0Speed2: TEdit;
    EditM1Speed: TEdit;
    EditM1Speed1: TEdit;
    EditM1Speed2: TEdit;
    EditUDPPort: TEdit;
    EditUDPSendPort: TEdit;
    ID_1: TEdit;
    ID_2: TEdit;
    ID_3: TEdit;
    EditIPR1: TEdit;
    EditIPR2: TEdit;
    EditIPR3: TEdit;
    Label1: TLabel;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    Label14: TLabel;
    Label15: TLabel;
    Label16: TLabel;
    Label17: TLabel;
    Label18: TLabel;
    Label19: TLabel;
    Label1_ID1: TLabel;
    Label1_ID2: TLabel;
    Label2: TLabel;
    Label20: TLabel;
    Label21: TLabel;
    Label22: TLabel;
    Label23: TLabel;
    Label24: TLabel;
    Label25: TLabel;
    Label26: TLabel;
    Label27: TLabel;
    Label28: TLabel;
    Label29: TLabel;
    Label3: TLabel;
    Label30: TLabel;
    Label31: TLabel;
    Label32: TLabel;
    Label33: TLabel;
    Label34: TLabel;
    Label35: TLabel;
    Label36: TLabel;
    Label37: TLabel;
    Label38: TLabel;
    Label39: TLabel;
    Label4: TLabel;
    Label40: TLabel;
    Label41: TLabel;
    Label42: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label1_ID: TLabel;
    Label1_IP: TLabel;
    Label2_IP: TLabel;
    Label3_IP: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    ShapeComBlink: TShape;
    UDPRobot: TLUDPComponent;
    UDP: TLUDPComponent;
    Serial: TSdpoSerial;
    ShapeSerialState: TShape;
    Robot_1: TLabel;
    Robot_2: TLabel;
    Robot_3: TLabel;
    ShapeUDPState: TShape;
    procedure Button2Click(Sender: TObject);
    procedure SendUDPClick(Sender: TObject);
    procedure UDPReceive(aSocket: TLSocket);
    procedure UDPRobotReceive(aSocket: TLSocket);
  //  procedure BCloseSerialClick(Sender: TObject);
 //   procedure BConfigSetClick(Sender: TObject);
  //  procedure BGoClick(Sender: TObject);
 //   procedure BOpenSerialClick(Sender: TObject);
  //  procedure BSendRawClick(Sender: TObject);
  //  procedure EditSendRawKeyDown(Sender: TObject; var Key: Word;
 //     Shift: TShiftState);
  //  procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
 //   procedure FormDestroy(Sender: TObject);
    procedure FormShow(Sender: TObject);
  //  procedure SerialRxData(Sender: TObject);
  private
  //  procedure Debug(s: string);
    procedure processFrame(channel: char; value: integer; source: integer; robot: integer);
    procedure SendUDPChannel(c: char; val: integer; i: integer);
  //  procedure ShowSerialState;
    procedure ShowUDPState;

    { private declarations }
  public
    BatVoltageTresh, BatVoltageConv: double;
    SerialChannels, UDPChannels: TChannels;
    BatVoltage: double;
    BatVoltageRaw, cont: integer;
    procedure SendMessage(c: char; val: integer);
  end;

var
  FRobot_Configuration: TFRobot_Configuration;
  pos_robot: Matrix2D; //de 0 a 2
  flagsign: boolean;
  state_Goto: integer;
  erro_theta_last: double;
  integrative: double;
  linear_vel, angular_vel: double;
  v1: IntArray;   //de 0 a 2
  v2: IntArray;
  Flag : integer;
  sign_previous : double;
  sign_dir : integer;
  sign_dir_f: integer;

implementation
 uses
   unit2,unit3, unit1, controlo;
{$R *.lfm}

function round2(const Number: extended; const Places: longint): extended;
var t: extended;
begin
  //Rounds a float value to X decimal points
   t := power(10, places);
   round2 := round(Number*t)/t;
end;


{ TFRobot_Configuration }

procedure TFRobot_Configuration.ShowUDPState;
begin
  if UDP.Connected then begin
    ShapeUDPState.Brush.Color := clGreen;
  end else begin
    ShapeUDPState.Brush.Color := clRed;
  end;
end;

procedure TFRobot_Configuration.SendMessage(c: char; val: integer);
begin
  Serial.WriteData(c + IntToHex(dword(Val), 8));
end;

procedure TFRobot_Configuration.SendUDPChannel(c: char; val: integer; i: integer);
var addr, ip: string;
begin
  if i=1 then begin
      ip:= EditIPR1.Text;
  end else if i=2 then begin
      ip:= EditIPR2.Text;
  end else if i=3 then begin
      ip:= EditIPR3.Text;
  end;
  addr := ip + ':' + EditUDPSendPort.Text;
  UDP.SendMessage(c + IntToHex(dword(Val), 8), addr);
end;

procedure TFRobot_Configuration.SendUDPClick(Sender: TObject);
  var s: string;
  v: integer;
  i: integer;
begin
  s := 'M';
  if s <> '' then begin
    v := StrToIntDef('12', 0);
    if (sender as TButton).Name = 'BSendR1' then begin
       i:=1;
    end else if (sender as TButton).Name = 'BSendR2' then begin
       i:=2;
    end else if (sender as TButton).Name = 'BSendR3' then begin
       i:=3;
    end;
    SendUDPChannel(s[1], v, i);
  end;
end;

procedure TFRobot_Configuration.Button2Click(Sender: TObject);
begin
  FRobot_Configuration.Hide;
  form1.Show;
end;

procedure TFRobot_Configuration.FormCreate(Sender: TObject);
begin
  DefaultFormatSettings.DecimalSeparator := '.';
  SerialChannels := TChannels.Create(@processFrame);
  UDPChannels := TChannels.Create(@processFrame);
  SetLength(pos_robot, 3, 4);
  SetLength(v1, 3);
  SetLength(v2, 3);
  //GOTOXYTHETA.Cancel := False;
end;

procedure TFRobot_Configuration.FormShow(Sender: TObject);
var port: integer;
begin
  //ShowSerialState();
  //BConfigSet.Click();
  //if CBAutoOpen.Checked then BOpenSerial.Click();

  port := StrToIntDef(EditUDPPort.Text, 9632);    //escuta mensagens vindas dos robots
  UDP.Listen(port);
  EditUDPPort.Text := IntToStr(port);
  ShowUDPState();
  UDPRobot.Listen(9600); //escuta mensagens vindas do rpi- sistema de localizacao
  EditIPR1.Text := '192.168.1.84';
  EditIPR2.Text := '192.168.1.83';
  EditIPR3.Text := '192.168.1.82';
end;

procedure TFRobot_Configuration.UDPReceive(aSocket: TLSocket);
var msg, Robot_IP: string;
  i: integer;
begin
  UDP.GetMessage(msg);
  if msg = '' then exit;

  Robot_IP := aSocket.PeerAddress;

  if Robot_IP = EditIPR1.Text then begin
       i:=1;
    end else if Robot_IP = EditIPR2.Text then begin
       i:=2;
    end else if Robot_IP = EditIPR3.Text then begin
       i:=3;
    end;

  UDPChannels.ReceiveData(msg, i);

  //if CBRawDebug.Checked then begin
  //  Debug(msg);
  //end;

end;


procedure TFRobot_Configuration.UDPRobotReceive(aSocket: TLSocket);
var
  msg, res, info, Robot_IP: string;
  n, i, id, k, robot: integer;
  A, aux: TStringArray;
  pos_x, pos_y, pos_z, pos_theta, gotoxy_robot, x_f, y_f, theta_f: integer;

begin

  UDPRobot.GetMessage(msg);

  if msg <> '' then begin
   n:=0;
     A := msg.Split(';'); //cada info de localizacao esta separada por ;

        for i:=0 to (length(A)-1) do
          begin

              aux := A[i].Split('x');
              res := aux[0];
              Delete(res, 1,1);
              id := StrToInt(res);

              if id = 43 then begin
                 n:=2;
              end
              else if id = 33 then begin
                 n:=1;
              end
              else if id = 23 then begin
                 n:=0;
              end;

        //rpi envia a posicao em centimetro- necessario passar para metros pq o mapa esta em metros
              aux := aux[1].Split('y');
              pos_robot[n][0]:= StrToInt(aux[0])/100;
              aux := aux[1].Split('z');
              pos_robot[n][1]:= (StrToInt(aux[0])/100);
              aux := aux[1].Split('t');
              pos_robot[n][2]:= StrToInt(aux[0])/100;
              pos_robot[n][3]:= degtorad(StrToInt(aux[1]));  //guardamos o angulo em radianos

          end;



   //prints

          Label16.Caption := 'Xini: ' + pos_robot[0][0].ToString;
          Label17.Caption := 'Yini: ' + pos_robot[0][1].ToString;
          Label18.Caption := 'Zini: ' + pos_robot[0][2].ToString;
          Label19.Caption := 'Thetaini: ' + (round2(pos_robot[0][3],2)).ToString;

          //Label19.Caption := 'id: 33- robot 2';
          Label21.Caption := 'Xini: ' + pos_robot[1][0].ToString;
          Label22.Caption := 'Yini: ' + pos_robot[1][1].ToString;
          Label23.Caption := 'Zini ' + pos_robot[1][2].ToString;
          Label24.Caption := 'Thetaini: ' + round2(pos_robot[1][3],2).ToString;

          //Label24.Caption := 'id: 43- robot 3';
          Label26.Caption := 'Xini: ' + pos_robot[2][0].ToString;
          Label27.Caption := 'Yini: ' + pos_robot[2][1].ToString;
          Label28.Caption := 'Zini ' + pos_robot[2][2].ToString;
          Label29.Caption := 'Thetaini: ' + round2(pos_robot[2][3],2).ToString;

          if (Button2.Visible=False) then begin

            FControlo.Label1.Caption := 'X: ' + (round2(form1.robots[0].pos_X,2)).ToString;
            FControlo.Label2.Caption := 'Y: ' + (round2(form1.robots[0].pos_Y,2)).ToString;
            FControlo.Label4.Caption := 'Direction: ' + (form1.robots[0].Direction).ToString;


            FControlo.Label5.Caption := 'X: ' + (round2(form1.robots[1].pos_X,2)).ToString;
            FControlo.Label6.Caption := 'Y: ' + (round2(form1.robots[1].pos_Y,2)).ToString;
            FControlo.Label8.Caption := 'Direction: ' + (form1.robots[1].Direction).ToString;


            FControlo.Label9.Caption := 'X: ' + (round2(form1.robots[2].pos_X,2)).ToString;
            FControlo.Label10.Caption := 'Y: ' + (round2(form1.robots[2].pos_Y,2)).ToString;
            FControlo.Label12.Caption := 'Direction: ' + (form1.robots[2].Direction).ToString;

          end;

          if (Button2.Visible=False) then begin          //significa que o form foi chamado já na fase de coordenaçao e não de configuração
                 //VerificaFalha(n);
                 //UpdateInitialPoints(n);
                flagVelocities:=true;
                FControlo.Controlo_Algoritmo(n);
          end;

  end;
end;

procedure TFRobot_Configuration.processFrame(channel: char; value: integer; source: integer; robot: integer);
var i: integer;
    s: string;
begin
  BatVoltageConv:= 0.003902912621359;
  //MemoDebug.Text := MemoDebug.Text + channel;
  if channel = 'i' then begin
  //end else if channel = 'r' then begin
    // if the arduino was reset ...
  end else if channel = 'V' then begin
    BatVoltageRaw := value;
    BatVoltage := value * BatVoltageConv;

    if robot = 1 then begin
      // EditBatVoltageRaw.Text := format('%d', [BatVoltageRaw]);
       EditBatVoltage.Text := format('%.2f', [BatVoltage]);
    end else if robot = 2 then begin
      // EditBatVoltageRaw1.Text := format('%d', [BatVoltageRaw]);
       EditBatVoltage1.Text := format('%.2f', [BatVoltage]);
    end else if robot = 3 then begin
     //  EditBatVoltageRaw2.Text := format('%d', [BatVoltageRaw]);
       EditBatVoltage2.Text := format('%.2f', [BatVoltage]);
    end;



      if ShapeComBlink.Brush.Color <> clGreen then begin
        ShapeComBlink.Brush.Color := clGreen;
      end else begin
        ShapeComBlink.Brush.Color := clLime;
      end;

  end else if channel = 'M' then begin //motor 1
    if robot =1 then begin
      EditM0Speed.Text := format('%d', [value]);
    end else if robot =2 then begin
      EditM0Speed1.Text := format('%d', [value]);
    end else if robot = 3 then begin
      EditM0Speed2.Text := format('%d', [value]);
    end;
  end else if channel = 'N' then begin  //motor 2
      if robot =1 then begin
      EditM1Speed.Text := format('%d', [value]);
    end else if robot =2 then begin
      EditM1Speed1.Text := format('%d', [value]);
    end else if robot = 3 then begin
      EditM1Speed2.Text := format('%d', [value]);
    end;
  end else if channel in ['s', 't'] then begin
    i := 1 + ord(channel) - ord('r');

  end;
end;

end.

