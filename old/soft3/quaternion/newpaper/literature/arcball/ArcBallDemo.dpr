program ArcBallDemo;

uses
  Forms,
  Main in 'Main.pas' {ABForm},
  arcball in 'arcball.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TABForm, ABForm);
  Application.Run;
end.
