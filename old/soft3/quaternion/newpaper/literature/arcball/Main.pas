unit Main;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, ArcBall, CgWindow, GL, Glut;

type
  TABForm = class(TCGForm)
    procedure FormCreate(Sender: TObject);
    procedure FormPaint(Sender: TObject);
    procedure FormMouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure FormMouseMove(Sender: TObject; Shift: TShiftState; X,
      Y: Integer);
    procedure FormMouseUp(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
  private
    { Private declarations }
  public
    { Public declarations }
  end;

var
  ABForm: TABForm;
  theBall: TArcBall;

implementation

{$R *.dfm}

procedure TABForm.FormCreate(Sender: TObject);
const
  LP: array [0..3] of Single = (0, 0, -1, 0);
begin

  try
    LoadGlut('glut32.dll');
    InitGL;

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, @LP);
    glEnable(GL_NORMALIZE);

    glLineWidth(3);
    glPointSize(10);

    theBall := TArcBall.Create;
    theBall.Init(256);
  except on E: Exception do
    begin
      MessageDlg(E.Message, mtError, [mbOk], 0);
      Halt(1);
    end;
  end;

end;

procedure TABForm.FormPaint(Sender: TObject);
begin

  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);
  glLoadIdentity;

  glLoadMatrixf(@theBall.Matrix);
  glutSolidTeapot(0.5);

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  theBall.Render;
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);

  PageFlip;

end;

procedure TABForm.FormMouseDown(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin

  if Button = mbLeft then
  begin
    theBall.BeginDrag;
    Paint;
  end;

end;

procedure TABForm.FormMouseMove(Sender: TObject; Shift: TShiftState; X,
  Y: Integer);
begin

  theBall.MouseMove(X, Y);
  Paint;

end;

procedure TABForm.FormMouseUp(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin

  if Button = mbLeft then
  begin
    theBall.EndDrag;
    Paint;
  end;

end;

end.
