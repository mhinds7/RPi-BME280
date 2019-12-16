
package RPi::BME280;

use strict;
use warnings;

our $VERSION = 0.0001;

use RPi::I2C;

sub new
{
    my ($class, %args) = @_;
    my $me = bless({ args=> \%args }, $class);
    $me->{addr} = $args{addr} // 0x76;
    $me->{dev} = RPi::I2C->new($me->{addr})
        || die("RPi::I2C->new($me->{addr}) failed");
    ($me->{devid} = $me->{dev}->read_byte(0xd0)) == 0x60
        || die("@{[ref($me)]} invalid ID: $me->{devid}");
    return $me;
}

sub reset
{
    my $me = $_[0];
    my $dev = $me->{dev};
    $me->{args}->{quiet} || printf(STDERR "@{[ref($me)]}::reset\n");

    $dev->write_byte(0xb6, 0x0e) == 0 # Reset
        || die("@{[ref($me)]}::reset failed");
    select(undef, undef, undef, 0.05);
    $me->setup();
}

sub setup
{
    my $me = $_[0];
    my $dev = $me->{dev};
    $me->{args}->{quiet} || printf(STDERR "@{[ref($me)]}::setup\n");

    my @rh_ctl   = (0x04, 0xf2); # 00000  101  0000 0100 OS=8
    my @meas_ctl = (0x91, 0xf4); # 100 100 01  1001 0001 OS=T8,P8,FORCE
    my @config   = (0x0c, 0xf5); # 000 011 00  0000 1100 INA=0.5ms,IIR=8

    if ($me->{args}->{verbose}) {
        printf("wr ctl-rh   %02x %02x\n", @rh_ctl);
        printf("wr ctl-meas %02x %02x\n", @meas_ctl);
        printf("wr config   %02x %02x\n", @config);
    }
    $dev->write_byte(@rh_ctl);
    $dev->write_byte(@meas_ctl);
    $dev->write_byte(@config);
}

sub init
{
    my ($me, %args) = @_;
    my $dev = $me->{dev};

    $me->{args}->{reset} && $me->reset();

    $me->{args}->{quiet} || printf(STDERR "@{[ref($me)]}::init\n");

    my $dig = $me->{dig} = {};
    ($dig->{T1}, $dig->{T2}, $dig->{T3}
    ,$dig->{P1}, $dig->{P2}, $dig->{P3}
    ,$dig->{P4} ,$dig->{P5}, $dig->{P6}
    ,$dig->{P7}, $dig->{P8}, $dig->{P9})
        = unpack('SssSs8', pack('C*', $dev->read_block(24, 0x88)));

    ($dig->{H1}, $dig->{H2}, $dig->{H3}, my ($E4, $E5, $E6), $dig->{H6})
        = unpack('CsCcCCc', pack('C*', $dev->read_byte(0xa1), $dev->read_block(7, 0xe1)));
    $dig->{H4} = ($E4<<4)|($E5&15);
    $dig->{H5} = ($E6<<4)|(($E5>>4)&15);

    my $params = {};
    ($params->{ctl_rh}, $params->{status}, $params->{ctl_meas}, $params->{config})
        = $dev->read_block(4, 0xf2);
    $me->{params} = $params;

    ($params->{ctl_meas} & 3) != 0
        || ($me->{trig} = [ $params->{ctl_meas} | 1, 0xf4 ]);

    if ($me->{args}->{verbose}) {
        printf("rd id       %02x d0\n", $me->{devid});
        printf("rd status   %02x f3\n", $params->{status});
        printf("rd ctl-rh   %02x f2\n", $params->{ctl_rh});
        printf("rd ctl-meas %02x f4\n", $params->{ctl_meas});
        printf("rd config   %02x f5\n", $params->{config});
    }
}

sub get
{
    my $me = $_[0];
    my $dev = $me->{dev};
    my $dig = $me->{dig};

    while ($me->{trig}) {
        $dev->write_byte(@{$me->{trig}});
        select(undef, undef, undef, 0.01);
        my $status = $dev->read_byte(0xf3);
        $me->{args}->{verbose} && printf(STDERR "@{[ref($me)]}::get status %02x\n", $status);
        if (($status & 0x11) == 0) {
            $me->{params}->{status} = $status;
            last;
        }
    }

    # Read the raw AD values
    my @v = $dev->read_block(8, 0xf7);
    my $at = ((($v[3]<<8)|$v[4])<<4)|($v[5]&15);
    my $rh = ($v[6]<<8)|$v[7];
    my $bp = ((($v[0]<<8)|$v[1])<<4)|($v[2]&15);

    # Compute AT
    my $var1 = (((($at>>3) - ($dig->{T1}<<1))) * $dig->{T2}) >> 11;
    my $var2 = ((((($at>>4) - $dig->{T1}) * (($at>>4) - $dig->{T1})) >> 12) * $dig->{T3}) >> 14;
    my $t_fine = $var1 + $var2;
    $at = 0.01 / 256 * ($t_fine * 5 + 128);
    if    ($at < -40.0) { $at = -40.1 }
    elsif ($at >  85.0) { $at =  85.1 }

    # Compute RH
    $rh = ($rh - ($dig->{H4} * 64 + $dig->{H5} / 16384 * ($t_fine - 76800)))
         * ($dig->{H2} / 65536 * (1 + $dig->{H6} / 67108864 * $rh * (1 + $dig->{H3} / 67108864 * $rh)));
    $rh = $rh * (1 - $dig->{H1} * $rh / 524288.0);
    if    ($rh > 100.0) { $rh = 100.1 }
    elsif ($rh <   0.0) { $rh =   0.0 }

#   # Compute BP - 32 bit
#   $var1 = $t_fine / 2.0 - 64000.0;
#   $var2 = $var1 * $var1 * $dig->{P6} / 32768.0;
#   $var2 = $var2 + $var1 * $dig->{P5} * 2.0;
#   $var2 = $var2 / 4.0 + $dig->{P4} * 65536.0;
#   $var1 = ($dig->{P3} * $var1 * $var1 / 524288.0 + $dig->{P2} * $var1) / 524288.0;
#   $var1 = (1.0 + $var1 / 32768.0) * $dig->{P1};
#   if ($var1 == 0) { $bp = 0.0 }
#   else {
#       $bp = 1048576.0 - $bp;
#       $bp = (($bp - $var2 / 4096.0) * 6250.0) / $var1;
#       $var1 = $dig->{P9} * $bp * $bp / 2147483648.0;
#       $var2 = $bp * $dig->{P8} / 32768.0;
#       $bp = 0.01 * ($bp + ($var1 + $var2 + $dig->{P7}) / 16.0);
#       if    ($bp > 1100.0) { $bp = 1100.1 }
#       elsif ($bp <  300.0) { $bp =  299.9 }
#   }

    # Compute BP - 64 bit
    $var1 = $t_fine - 128000;
    $var2 = $var1 * $var1 * $dig->{P6};
    $var2 = $var2 + $var1 * $dig->{P5} * 131072;
    $var2 = $var2 + $dig->{P4} * 34359738368;
    $var1 = $var1 * $var1 * $dig->{P3} / 256 + $var1 * $dig->{P2} * 4096;
    $var1 = (16384 + $var1 / 8589934592) * $dig->{P1};
    if ($var1 == 0) { $bp = 0.0 }
    else {
        my $var4 = 1048576 - $bp;
        $var4 = ($var4 * 2147483648 - $var2) * 3125 / $var1;
        $var1 = $dig->{P9} * $var4 * $var4 / 2249051034615808;
        $var2 = $dig->{P8} * $var4 / 524288; 
        $var4 = ($var4 + $var1 + $var2) / 256 + $dig->{P7} * 16;
        $bp = 0.01 / 256 * $var4;
        if    ($bp <  300.0) { $bp =  299.9 }
        elsif ($bp > 1100.0) { $bp = 1100.1 }
    }

    return ($at, $rh, $bp);
}

1;
__END__

=head1 NAME

RPi::BME280 - Interface to the BME280 AT/RH/BP sensor

=head1 SYNOPSIS

    use RPi::BMP180;

    my $bme = RPi::BME280->new(keyval list) || die("RPi::BMP180::new failed");

    $bme->reset() || die("RPi::BMP180::reset failed");

    (my ($at, $rh, $bp) = $bme->get()) || die("RPi::BMP180::get failed");

=head1 DESCRIPTION

This module allows you to interface with a BME280 AT/RH/BP sensor.

=head1 METHODS

=head2 new(<keyval list>)
reset   => <not true> or true the reset BME280
quiet   => <not true> or print on info messages to STDERR
verbose => <not true> or print on extra info messages to STDERR
loop    => 0 or interval time in seconds

Returns a new C<RPi::BME280> object.

=head2 reset

Reset the sensor and setup the measurement parameters.

Parameters:
    none

=head2 init

Read the calibration data from the sensor for later use by get().

Parameters:
    none

=head2 get

Fetches the AT, RH, BP values from the sensor.

Parameters:
    none

=head1 AUTHOR

Mark Hinds, E<lt>zoro98020@gmail.comE<gt>

=head1 COPYRIGHT AND LICENSE

Copyright (C) 2019 by Mark Hinds

This library is free software; you can redistribute it and/or modify
it under the same terms as Perl itself, either Perl version 5.18.2 or,
at your option, any later version of Perl 5 you may have available.

