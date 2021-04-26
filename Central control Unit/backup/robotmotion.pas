unit robotmotion;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils;

var

implementation
 uses
   unit1,unit2;
{$R *.lfm}


function GoTo_XY(i:integer; xf, yf, tf : double): matrix;
    var
      k1, k2:double;
      v_go: double;
      tol_t, tol_pos: double;

    Begin
      k1:= 60;
      k2:= 50;
      v_go := 10;
      tol_t := 0.03;
      tol_pos := 0.03;
      Result:= Mzeros(1,1);

      x_k:= GetRobotX(i-1);
      y_k:= GetRobotY(i-1);
      theta_k := GetRobotTheta(i-1);
      erro_ang_goto[i] := DiffAngle(ang_ida[i], theta_k);
      erro_ang_final[i] := DiffAngle(tf, theta_k);
      SetRCValue(33, 5 ,format('%g',[ang_ida[i]]));
      SetRCValue(34, 5 ,format('%g',[abs(erro_ang_goto[i])]));
      SetRCValue(35, 5 ,format('%g',[abs(erro_ang_final[i])]));

      case state[i] Of
      0:begin
      SetRCValue(40, 5 ,'Ainda_zero');
      ang_ida[i] := atan2(yf-y_k, xf-x_k);
      state[i] := 1;
      if ((((x_k+tol_pos)>xf) and ((x_k-tol_pos)<xf)) and (((y_k+tol_pos)>yf) and ((y_k-tol_pos)<yf))  and (((theta_k+tol_t)>tf) and ((theta_k-tol_t)<tf))) then begin
              state[i]:= 4;
            end;
      end;
        //Rota??o para posicionamente de partida
        1:begin
             if ((((x_k+tol_pos)>xf) and ((x_k-tol_pos)<xf)) and (((y_k+tol_pos)>yf) and ((y_k-tol_pos)<yf))  and (((theta_k+tol_t)>tf) and ((theta_k-tol_t)<tf))) then begin
              state[i]:= 4;
            end else begin
            Velocity := VelocityCalc_GoTo(i,0, k1*erro_ang_goto[i]);
            SetRCValue(40, 5 ,'Ainda_1')
            if(abs(erro_ang_goto[i])< tol_t) then begin
            SetRCValue(40, 6 ,'state1_erro');
            state[i]:= 2;
            SetAxisSpeedRef(i-1, 0, 0);  //V1
            SetAxisSpeedRef(i-1, 1, 0);  //V2
            end;
            end;
          end;

          //Avan?ar para a posi??o
        2:begin
          //Ocorrencia de erro
          SetRCValue(32, 5 ,format('%d',[state[i]]));
          SetRCValue(40,5,'state2');
          SetRCValue(41, 5 ,'Ainda_2')
           (* if(abs(erro_ang_goto)>0.05) then begin
            state := 1;
            SetRCValue(41, 7 ,'secalhar')
            end;    *)

           Velocity := VelocityCalc_GoTo(i,v_go, k2*erro_ang_goto[i]);
           SetRCValue(41, 4 ,'aqui')
             if((abs(xf-x_k)< tol_pos) and (abs(yf-y_k)< tol_pos))then begin
               state[i]:=3;
               SetAxisSpeedRef(i-1, 0, 0);
               SetAxisSpeedRef(i-1, 1, 0);
             end;
          end;

          //Rodar para o ?ngulo final
          3:begin
         SetRCValue(32, 5 ,format('%d',[state[i]]));

              if(abs(erro_ang_final[i])> tol_t) then begin
              Velocity := VelocityCalc_GoTo(i,0, k2*erro_ang_final[i]);
              end;
           SetRCValue(40,5,'state3');
              if(abs(erro_ang_final[i])< tol_pos) then begin
              state[i]:=4;
              end;
          end;
          4:begin
          SetRCValue(32, 5 ,format('%d',[state[i]]));
          aux:= 0;
          SetRCValue(40,5,'state4');
          //Nothing
          end;
    end;
end;

end.

