use fixed::types::I22F10;
//An first order butterworth filter
// const FILTER_ORDER:u16 = 1;//the order of butterworth filter
const CUT_OFF_FREQ:u16 = 10;//the cut-off frequency of butterworth filter

const SAMPLING_FREQ:u16 = 100;//the sample frequency of butterworth filter
pub struct ButterWorth{
    pub cutoff_freq: I22F10,
    pub sampling_freq: I22F10,
    pub a1: I22F10,
    pub b0: I22F10,
    pub b1: I22F10,
    pub prev_input: I22F10,
    pub prev_output: I22F10,
}
///First order butterworth filter
///Formula(sampling and discrete): y[n]=b0*x[n]+b1*x[n−1]−a1*y[n−1]
///x[n]: current sample input
///x[n-1]: previous sample input
///y[n-1]: previous sample output
/// fs:sampling frequency
/// fc:cut_off frequency
/// Wc:normalize cut_off_frequency: 2*pi*fc
/// dt: period of sampling: 1/(2*pi*fs)
/// a0: (Wc-dt)/(Wc+dt)
/// b0: Wc/(Wc+dt)
/// b1: Wc/(Wc+dt)
impl ButterWorth {
    pub fn new() -> Self {
        let mut butterworth = ButterWorth {
            cutoff_freq:I22F10::from_num(CUT_OFF_FREQ),
            sampling_freq:I22F10::from_num(SAMPLING_FREQ),
            a1: I22F10::from_num(0),
            b0: I22F10::from_num(0),
            b1: I22F10::from_num(0),
            prev_input: I22F10::from_num(0),
            prev_output: I22F10::from_num(0),
        };
        butterworth.calculate_coefficients();
        butterworth
    }

    fn calculate_coefficients(&mut self) {
        let n = self.sampling_freq/self.cutoff_freq;
        self.b0 = I22F10::from_num(1)/(I22F10::from_num(2)*n) ;
        self.b1 = I22F10::from_num(1)/(I22F10::from_num(2)*n);
        self.a1 = I22F10::from_num(1) - (I22F10::from_num(1)/n);
    }

    pub fn filter(&mut self, input: I22F10) -> I22F10 {
        let output = self.b0 * input + self.b1 * self.prev_input + self.a1 * self.prev_output;
        self.prev_input = input;
        self.prev_output = output;
        output
    }
}