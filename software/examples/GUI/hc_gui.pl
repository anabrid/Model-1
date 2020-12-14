#
#  Simple GUI to control an Analog Paradigm Model-1 analog computer 
# using the hybrid controller module HC and the IO::HyCon perl module.
#
# 24-JAN-2020   B. Ulmann   Initial version
#

use strict;
use warnings;

use 5.010;
use Wx;

package MyFrame;

use Data::Dumper;
use base 'Wx::Frame';
use Wx::Event qw(EVT_BUTTON);
use IO::HyCon;

my $version = 'v. 0.1';

sub new {
    my ($ref) = @_;

    my $ac = IO::HyCon->new();
    $ac->disable_ext_halt();

    my ($ic_time, $op_time) = (10, 100);    # IC and OP times in milliseconds
    my $address = "0000";

    my $self = $ref->SUPER::new(undef,  # parent window
        -1,                 # ID -1 means any
        "HC GUI $version",  # title
        [-1, -1],           # default position
        [600, 650],         # size
    );
    my $panel = Wx::Panel->new($self, -1);

    # Create eight buttons to control the digital outputs
    my ($x, $y, $y_inc) = (30, 20, 40);
    for my $i (0 .. 7) {
        my $dout_button = Wx::Button->new($panel,
            $i, "D$i = 0", [$x, $y + $i * $y_inc], [-1, -1]);
        Wx::Event::EVT_BUTTON($dout_button, -1, sub {
            my ($button, $event) = @_;
            state $status = 0;
            $status = 1 - $status;
            $ac->digital_output($i, $status);
            $button->SetTitle("D$i = $status");
        });

        $ac->digital_output($i, 0);
    }

    #  Create sliders for the digital potentiometers, these must be defined in the
    # elements-section as DPT0..DPT7 and can be mapped to any digital potentiometers
    # in the system, including those in the HC but also on additional DPT24 modules.
    my $x_inc;
    ($x, $x_inc, $y, $y_inc) = (35, 260, 365, 40);
    for my $i (0 .. 7) {
        my $s_value = Wx::StaticText->new($panel, -1, "DPT$i: 0.0000", [$x, $y]);
        my $slider  = Wx::Slider->new($panel, -1, 0, 0, 1023, [$x + 95, $y + 5], [150, 10]);
        Wx::Event::EVT_SLIDER($slider, -1, sub {
            my ($slider, $event) = @_;
            my $value = sprintf("%0.4f", $slider->GetValue() / 1023);
            $s_value->SetLabel("DPT$i: $value");
            $ac->set_pt("DPT$i", $value);
        });

        $ac->set_pt("DPT$i", 0);

        if ($i == 3) {  # Second column
            $x += $x_inc;
            $y -= 3 * $y_inc;
        } else {
            $y += $y_inc;
        }
    }

    # Create IC/OP/HALT buttons
    my $ic_button   = Wx::Button->new($panel, -1, 'IC',   [150, 20], [-1, -1]);
    my $op_button   = Wx::Button->new($panel, -1, 'OP',   [300, 20], [-1, -1]);
    my $halt_button = Wx::Button->new($panel, -1, 'HALT', [450, 20], [-1, -1]);

    # Create Single-Run- and RepOp-buttons
    my $sr_button = Wx::Button->new($panel, -1, 'Single Run', [450, 100], [-1, -1]);
    my $ro_button = Wx::Button->new($panel, -1, 'Rep Op',     [450, 140], [-1, -1]);

    # Create readout buttons
    my $readout_button = Wx::Button->new($panel, -1, 'Readout', [450, 220], [-1, -1]);
    my $din_button     = Wx::Button->new($panel, -1, 'Digital in', [450, 260], [-1, -1]);

    # Create misc. buttons
    my $enable_ovl_button  = Wx::Button->new($panel, -1, 'OvlHlt = 0', [150, 540], [-1, -1]);
    my $enable_ehlt_button = Wx::Button->new($panel, -1, 'ExtHlt = 0', [300, 540], [-1, -1]);
    my $quit_button        = Wx::Button->new($panel, -1, 'Quit', [450, 540], [-1, -1]);
 
    # Create entry fields for IC and OP times for single runs and repetitive operation etc.
    my $ic_time_label = Wx::StaticText->new($panel, -1, 'IC-time (ms):', [160, 105]);
    my $op_time_label = Wx::StaticText->new($panel, -1, 'OP-time (ms):', [160, 145]);
    my $address_label = Wx::StaticText->new($panel, -1, 'Address (hex):', [160, 225]);
    my $readout_field = Wx::StaticText->new($panel, -1, 'Typ: ???  Val: N/A', [300, 225]);
    my $din_field     = Wx::StaticText->new($panel, -1, 'Digital input: - - - - - - - -', [160, 265]);
    my $status_field  = Wx::StaticText->new($panel, -1, 'Status: OK', [160, 305]);

    my $ic_time_field = Wx::TextCtrl->new($panel, -1, $ic_time, [250, 102]);
    my $op_time_field = Wx::TextCtrl->new($panel, -1, $op_time, [250, 142]);
    my $address_field = Wx::TextCtrl->new($panel, -1, $address, [250, 222], [40, -1]);

    # Defaul settings for the buttons
    $ic_button->Enable();
    $op_button->Disable();
    $halt_button->Disable();
    $sr_button->Enable();
    $ro_button->Enable();

    # Create interlocks etc.
    Wx::Event::EVT_BUTTON($ic_button, -1, sub {     # IC
        my ($button, $event) = @_;
        $ic_button->Disable();
        $op_button->Enable();
        $halt_button->Enable();
        $sr_button->Enable();
        $ro_button->Enable();
        $status_field->SetLabel('Status: IC');
        $ac->ic();
    });

    Wx::Event::EVT_BUTTON($op_button, -1, sub {     # OP
        my ($button, $event) = @_;
        $ic_button->Enable();
        $op_button->Disable();
        $halt_button->Enable();
        $sr_button->Disable();
        $ro_button->Disable();
        $status_field->SetLabel('Status: OP');
        $ac->op();
    });

    Wx::Event::EVT_BUTTON($halt_button, -1, sub {   # HALT
        my ($button, $event) = @_;
        $ic_button->Enable();
        $op_button->Enable();
        $halt_button->Disable();
        $sr_button->Disable();
        $ro_button->Disable();
        $status_field->SetLabel('Status: HALT');
        $ac->halt();
    });

    Wx::Event::EVT_BUTTON($sr_button, -1, sub {     # Single-Run
        my ($button, $event) = @_;

        ($ic_time, $op_time) = ($ic_time_field->GetValue(), $op_time_field->GetValue());
        if ($ic_time !~ /^\d+$/ or $ic_time > 999999) {
            $status_field->SetLabel('Status: Illegal IC-time!');
        } elsif ($op_time !~ /^\d+$/ or $op_time > 999999) {
            $status_field->SetLabel('Status: Illegal OP-time!');
        } else {
            $status_field->SetLabel('Status: Single Run');
            $ic_button->Enable();
            $op_button->Disable();
            $halt_button->Disable();

            $ac->set_ic_time($ic_time);
            $ac->set_op_time($op_time);
            $ac->single_run();

            $sr_button->Enable();
            $ro_button->Enable();
        }
    });

    Wx::Event::EVT_BUTTON($ro_button, -1, sub {     # Repetitive operation
        my ($button, $event) = @_;

        ($ic_time, $op_time) = ($ic_time_field->GetValue(), $op_time_field->GetValue());
        if ($ic_time !~ /^\d+$/ or $ic_time > 999999) {
            $status_field->SetLabel('Status: Illegal IC-time!');
        } elsif ($op_time !~ /^\d+$/ or $op_time > 999999) {
            $status_field->SetLabel('Status: Illegal OP-time!');
        } else {
            $status_field->SetLabel('Status: Rep Op');
            $ic_button->Enable();
            $op_button->Disable();
            $halt_button->Enable();

            $ac->set_ic_time($ic_time);
            $ac->set_op_time($op_time);
            $ac->repetitive_run();
        }
    });

    # Setup Readout functionality
    Wx::Event::EVT_BUTTON($readout_button, -1, sub {    # Readout
        my ($button, $event) = @_;
        my $address = $address_field->GetValue();
        if ($address !~ /^[0-9A-F]{1,4}$/i) {
            $status_field->SetLabel("Status: Illegal address ($address)");
        } else {
            my $result = $ac->read_element_by_address(hex($address));
            my $id = $result->{id} !~ /UNKNOWN/ ? $result->{id} : '???';
            $readout_field->SetLabel(sprintf("Typ: %4s Val: %0.4f", $id, $result->{value}));
        }
    });

    Wx::Event::EVT_BUTTON($din_button, -1, sub {        # Data input
        my ($button, $event) = @_;
        $din_field->SetLabel("Digital input: " . join(' ', @{$ac->read_digital()}));
    });

    Wx::Event::EVT_BUTTON($enable_ovl_button, -1, sub { # Enable halt on overload
        my ($button, $event) = @_;
        state $status = 0;
        $status = 1 - $status;
        $enable_ovl_button->SetLabel("OvlHlt = $status");
        $status ? $ac->enable_ovl_halt() : $ac->disable_ovl_halt();
    });

    Wx::Event::EVT_BUTTON($enable_ehlt_button, -1, sub {    # Enable external halt input
        my ($button, $event) = @_;
        state $status = 0;
        $status = 1 - $status;
        $enable_ehlt_button->SetLabel("ExtHlt = $status");
        $status ? $ac->enable_ext_halt() : $ac->disable_ext_halt();
    });

    Wx::Event::EVT_BUTTON($quit_button, -1, sub {  # Quit the application
        $ac->reset();
        $self->Close();
    });

    return $self;
}

package MyApp;

use base 'Wx::App';

sub OnInit {
    my $frame = MyFrame->new;
    $frame->Show(1);
}

package main;

my $app = MyApp->new;
$app->MainLoop;
