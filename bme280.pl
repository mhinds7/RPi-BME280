#! /usr/bin/perl

use RPi::BME280;

$_ = join(' ', @ARGV);
if (/\s*--help\s*/ || /\s*-h\*/) {
    print(STDERR <<EOD);
usage: bme280.pl [-q] [-v] [reset] [loop=<interval>]
  -q              Be Quiet
  -v              Be Verbose
  reset           Reset the BME280 and setup measurement params
  loop=<interval> Produce a line of AT.RH.BP values once per <interval>
EOD
    exit(0);
}

my %ARGS;
$ARGS{reset} = s/reset\s*// ? 1 : '';
$ARGS{setup} = s/setup\s*// ? 1 : '';
$ARGS{loop} = s/loop=(\S+)\s*// ? $1 : 0;
$ARGS{debug} = s/-d\s*// ? 1 : '';
$ARGS{verbose} = s/-v\s*// ? 1 : '';

my $bme280 = RPi::BME280->new(%ARGS);
$bme280->init();

if (!$bme280->{args}->{loop}) {
    my ($at, $rh, $bp) = $bme280->get();
    printf("at : %.2f C\n", $at);
    printf("rh : %.2f %%\n", $rh);
    printf("bp : %.4f hPa\n", $bp);
}
else {
    require Event;
    select(STDOUT); $| = 1;
    Event->timer(
        at      => int(Event::time()+1.0),
        interval=> $bme280->{args}->{loop},
        hard    => 1,
        cb      => sub {
            my ($t, $at, $rh, $bp) = (Event::time(), $bme280->get());
            printf("%.3f %6.2f C %6.2f %% %9.4f hPa\n", $t, $at, $rh, $bp);
    });
    Event::loop();
}

