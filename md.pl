#!/usr/bin/perl
use strict;
use POSIX;
use IO::Select;
use IO::Handle;
use Term::Cap;
use Term::ReadKey;
use Time::HiRes qw(usleep);
use Term::ANSIColor qw(colored color :constants );
use Term::ReadKey;


#inverse r coeff simulates gravity in 2d
my $inverse_r_coef =  +180;
#inverse r2 force keeps bodies from collapsing into singularity
my $inverse_r2_coef =  -900;
#drag makes it a little more visually appealing
my $drag_coef = 0.08000;
#COR for collision w/ walls
my $wallcoef = 0.9;
my $numballs = 100;
#(approx) time step and #of frames to skip drawing
my $dt = 0.00002;
my $skip_factor = 00;


#unbuffer output, stdin uncooked
$|++;
ReadMode 3;


#terminal dimensions
my ($wchar, $hchar, $wpix, $hpix) = GetTerminalSize();
my $t = Term::Cap->Tgetent;
my $maxx = $wchar;
my $maxy = $maxx;
my $yscale = $wchar/$hchar;

#arrays for body properties
my @xpos;
my @ypos;
my @xvel;
my @yvel;
my @xf;
my @yf;
my @mass;
my @objstr;
#indicies of larger bodies
my @bigun_idxs = (4,5);


#initialization
cls();
init();
my $skip = 0;
#main loop
while (1)  {
    #process interactive controls
    if (my $txt = ReadKey(-1)){
        init() if $txt eq 'r';
        if ($txt eq 'j'){
            for my $idx (@bigun_idxs){
                $mass[$idx] *= 1.2;
            }
        }elsif ($txt eq 'k'){
            for my $idx (@bigun_idxs){
                $mass[$idx] /= 1.2;
            }
        }elsif ($txt eq 'h'){
            for my $idx (0..$numballs-1){
                $mass[$idx] *= 1.2;
            }
        }elsif ($txt eq 'l'){
            for my $idx (0..$numballs-1){
                $mass[$idx] /= 1.2;
            }
        }elsif ($txt eq 'd'){
            $drag_coef *= 1.2;
        }elsif ($txt eq 'c'){
            $drag_coef /= 1.2;
        }elsif ($txt eq 'g'){
            $inverse_r2_coef *= 1.2;
        }elsif ($txt eq 'b'){
            $inverse_r2_coef /= 1.2;
        }elsif ($txt eq 'f'){
            $inverse_r_coef *= 1.2;
        }elsif ($txt eq 'v'){
            $inverse_r_coef /= 1.2;
        }elsif ($txt eq 's'){
            $wallcoef *= 1.2;
        }elsif ($txt eq 'x'){
            $wallcoef /= 1.2;
        }elsif ($txt eq 'q'){
            ReadMode 1;
            exit();
        }
    }
    #skip drawing <$skip_factor> frames 
    if($skip++ >= $skip_factor){
        cls();
        $skip = 0;
        for my $i  (0..$numballs-1){
            drawdot($xpos[$i], $ypos[$i], $objstr[$i]);
        }
        draw_titlebar();
        print $t->Tputs('ho', 1);
    }
    
    #update velocities and positions
    for my $i  (0..$numballs-1){
        $xvel[$i] += $dt*$xf[$i]/($mass[$i]);
        $yvel[$i] += $dt*$yf[$i]/($mass[$i]);

        $xpos[$i] += ($dt*$xvel[$i]);
        if($xpos[$i] > $maxx){
            $xpos[$i] =$maxx -1;
            $xvel[$i] *=-$wallcoef;
            $xf[$i] =0;
        }
        if($xpos[$i] < 0){
            $xpos[$i] =1;
            $xvel[$i] *= -$wallcoef;
            $xf[$i] =0;
        }

        $ypos[$i] += ($dt*$yvel[$i]);
        if($ypos[$i] > $maxy){
            $ypos[$i] =$maxy -1;
            $yvel[$i] *=-$wallcoef;
            $yf[$i] =0;
        }
        if($ypos[$i] < 0){
            $ypos[$i] =1;
            $yvel[$i] *=-$wallcoef;
            $yf[$i] =0;
        }
    }

    #recompute forces
    compute_forces();

    #delay loop
    usleep($dt*1000000);
}


#initialize bodies
sub init{
    for my $i  (0..$numballs-1){
        $xpos[$i] = rand($maxx);
        $ypos[$i] = rand($maxy);
        $xvel[$i] = rand(1)/10.0;
        $yvel[$i] = rand(1)/10.0;
        $xf[$i] = 0;
        $yf[$i] = 0;
        $mass[$i] = 11;
        $objstr[$i] = colored(".", 'bold blue');
    }
    my @colorz = qw(green red yellow white);
    for my $idx (@bigun_idxs){
        $mass[$idx] =  +1000000;
        $objstr[$idx] = colored("o", 'bold '. $colorz[($idx)%(scalar @colorz)]);
    }
    #$mass[5] =  +1000000;
    #$objstr[5] = colored("o", 'bold red');
    #$mass[6] =  300;
    #$objstr[6] = colored("@", 'bold white');
    #$mass[7] =  500;
    #$objstr[7] = colored("@", 'bold yellow');
}

#draw titlebar showing physical constants and masses
sub draw_titlebar{
    print $t->Tgoto("cm",0,0);
    
    #my $string = sprintf("inv_r_coef=%g). \t inv_r2_coef=%g \t drag_coef=%g \t wall_cor=%g \t big_mass=%g \t small_mass=%g ", $inverse_r_coef, $inverse_r2_coef, $drag_coef, $wallcoef, $mass[$bigun_idxs[0]], $mass[$numballs-1]);
    my @fields = (sprintf(  "inv_r_coef=%g", $inverse_r_coef),
                 sprintf(  "inv_r2_coef=%g", $inverse_r2_coef),
                 sprintf(  "drag_coef=%g", $drag_coef), 
                 sprintf(  "wall_cor=%g", $wallcoef), 
                 sprintf(  "big_mass=%g", $mass[$bigun_idxs[0]]), 
                 sprintf(  "small_mass=%g ",$mass[$numballs-1]));
    my $string = '';
    for my $f (@fields){
        $string .= sprintf("%30s ", $f);
    }
    my @stringz = $string =~ /(.{1,$wchar}[^\w=])/g ;
    foreach my $str (@stringz){
        print( BLACK . ON_WHITE . BOLD . sprintf("%-*s\n", $wchar, $str) . RESET);
        #print(colored(sprintf("%.*s\n", $wchar, $str), "on_blue black"));
    }
}

#compute teh forces on bodies
sub compute_forces{
    for my $i (0..$numballs-1){
        $xf[$i] = 0;
        $yf[$i] = 0;
    }
    for my $i (0..$numballs-1){
        for my $j ($i+1..$numballs-1){
            next if $j == $i;
            my $dx = ($xpos[$i]-$xpos[$j]);
            my $dy = ($ypos[$i]-$ypos[$j]);
            my $rs = sqrt($dx**2 + $dy**2);
            $rs = 1e-200 if $rs == 0;
            #gravity
            my $f = $inverse_r_coef * ($mass[$i] * $mass[$j])/ ($rs);
            my $f2 = $inverse_r2_coef * abs($mass[$i] * $mass[$j])/ (($rs*sqrt($rs)));
            $f += $f2;
            $xf[$i] -= $f*$dx;
            $yf[$i] -= $f*$dy;
            $xf[$j] += $f*$dx;
            $yf[$j] += $f*$dy;
        }
        my $vrms = sqrt(($yvel[$i] + $dt*$yf[$i]/($mass[$i]))**2 + ($xvel[$i] + $dt*$xf[$i]/($mass[$i]))**2);
        #$yvel[$i] += -$drag_coef*($yvel[$i])*$vrms; 
        #$xvel[$i] += -$drag_coef*($xvel[$i])*$vrms; 
        $yf[$i] += -$drag_coef*($yvel[$i] + $dt*$yf[$i]/abs($mass[$i]))*$vrms; 
        $xf[$i] += -$drag_coef*($xvel[$i] + $dt*$xf[$i]/abs($mass[$i]))*$vrms; 
    }
}

#draw body at a given position
sub drawdot {
    my ($x, $y, $str) = @_;
    print $t->Tgoto("cm",int($x),int(1+$y/$yscale));
    print $str;
}

#clear the screen
sub cls {
    print $t->Tputs('cl', -1);
    draw_titlebar();
    #system(clear);
}

