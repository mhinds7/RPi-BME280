use 5.006;
use strict;
use warnings;
use ExtUtils::MakeMaker;

WriteMakefile(
    NAME             => 'RPi::BME280',
    AUTHOR           => q{Mark Hinds <zoro98020@gmail.com>},
    VERSION_FROM     => 'lib/RPi/BME280.pm',
    ABSTRACT_FROM    => 'lib/RPi/BME280.pm',
    LICENSE          => 'perl_5',
    LIBS             => [],
    PL_FILES         => {},
    MIN_PERL_VERSION => 5.006,
    CONFIGURE_REQUIRES => {
        'RPi::I2C' => 0,
    },
    BUILD_REQUIRES => {
        'Test::More' => 0,
    },
    PREREQ_PM => {
        'RPi::I2C'           => 2.3608,
        'Event'              => '1.24',
    },
    dist  => { COMPRESS => 'gzip -9f', SUFFIX => 'gz', },
    clean => { FILES => 'RPi-BME280-*' },
    EXE_FILES => [ 'bme280.pl' ],
);
