#!perl -T
use 5.006;
use strict;
use warnings;
use Test::More;

plan tests => 1;

BEGIN {
    use_ok( 'RPi::BME280' ) || print "Bail out!\n";
}

diag( "Testing RPi::BME280 $RPi::BME280::VERSION, Perl $], $^X" );
