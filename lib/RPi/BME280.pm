
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
        || die("@{[ ref($me) ]} invalid ID: $me->{devid}");
    return $me;
}

sub reset
{
    my $me = $_[0];
    my $dev = $me->{dev};
    my $class = ref($me);

    $me->{args}->{quiet} || printf(STDERR "${class}::reset\n");
    $dev->write_byte(0xb6, 0x0e) == 0 # Reset
        || die("${class}::reset failed");

    select(undef, undef, undef, 0.1);

    $me->{args}->{quiet} || printf(STDERR "${class} Setup Meas Params\n");
    $dev->write_byte(0x01, 0xf2);
    $dev->write_byte(0x27, 0xf4); # 001 001 11 0010 0111 0x27
    $dev->write_byte(0x0c, 0xf5); # 000 011 00 0000 1100 0x0c
}

sub init
{
    my ($me, %args) = @_;
    my $dev = $me->{dev};

    $me->{args}->{quiet} || printf(STDERR "@{[ ref($me) ]}::init\n");

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

    $me->{args}->{reset} && $me->reset();

    if ($me->{args}->{verbose}) {
        printf("id       0xd0 %02x\n", $me->{devid});
        printf("status   0xf3 %02x\n", $params->{status});
        printf("ctl-rh   0xf2 %02x\n", $params->{ctl_rh});
        printf("ctl-meas 0xf4 %02x\n", $params->{ctl_meas});
        printf("config   0xf5 %02x\n", $params->{config});
    }
}

sub get
{
    my $me = $_[0];
    my $dev = $me->{dev};
    my $dig = $me->{dig};

    # Read the raw AD values
    my @v = $dev->read_block(8, 0xf7);
    my $at = ((($v[3]<<8)|$v[4])<<4)|($v[5]&15);
    my $rh = ($v[6]<<8)|$v[7];
    my $bp = ((($v[0]<<8)|$v[1])<<4)|($v[2]&15);

    # Compute AT
    my $var1 = (((($at>>3) - ($dig->{T1}<<1))) * $dig->{T2}) >> 11;
    my $var2 = ((((($at>>4) - $dig->{T1}) * (($at>>4) - $dig->{T1})) >> 12) * $dig->{T3}) >> 14;
    my $t_fine = $var1 + $var2;
    $at = 0.01 * (($t_fine * 5 + 128) >> 8);

    # Compute RH
    $rh = ($rh - ($dig->{H4} * 64.0 + $dig->{H5} / 16384.0 * ($t_fine - 76800.0)))
         * ($dig->{H2} / 65536.0 * (1.0 + $dig->{H6} / 67108864.0 * $rh * (1.0 + $dig->{H3} / 67108864.0 * $rh)));
    $rh = $rh * (1.0 - $dig->{H1} * $rh / 524288.0);
    if    ($rh > 110) { $rh = 110 }
    elsif ($rh <   0) { $rh =   0 }

    # Compute BP
    $var1 = $t_fine / 2.0 - 64000.0;
    $var2 = $var1 * $var1 * $dig->{P6} / 32768.0;
    $var2 = $var2 + $var1 * $dig->{P5} * 2.0;
    $var2 = $var2 / 4.0 + $dig->{P4} * 65536.0;
    $var1 = ($dig->{P3} * $var1 * $var1 / 524288.0 + $dig->{P2} * $var1) / 524288.0;
    $var1 = (1.0 + $var1 / 32768.0) * $dig->{P1};
    if ($var1 == 0) { $bp = 0 }
    else {
        $bp = 1048576.0 - $bp;
        $bp = (($bp - $var2 / 4096.0) * 6250.0) / $var1;
        $var1 = $dig->{P9} * $bp * $bp / 2147483648.0;
        $var2 = $bp * $dig->{P8} / 32768.0;
        $bp = 0.01 * ($bp + ($var1 + $var2 + $dig->{P7}) / 16.0);
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

