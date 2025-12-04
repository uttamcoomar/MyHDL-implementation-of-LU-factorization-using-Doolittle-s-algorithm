"""
Strict VHDL-convertible LU decomposition (Doolittle) with Q8.24 fixed-point
Divider (LUT + Newton-Raphson). Warning: No pivot check!

"""

from myhdl import *

# --------------------------------------------------------------------------
# Compile-time constant matrix size
# --------------------------------------------------------------------------
N = 4   # <<< SET MATRIX SIZE HERE (compile-time constant)

# --------------------------------------------------------------------------
# Fixed-point configuration (module-level constants OK)
# --------------------------------------------------------------------------

FRAC_BITS = 24
INT_BITS = 8
WORD_WIDTH = INT_BITS + FRAC_BITS # 32
ELEMENT_BITS = WORD_WIDTH
NUM_ELEM = N * N
FLAT_WIDTH = ELEMENT_BITS * NUM_ELEM
MASK_32 = (1 << ELEMENT_BITS) - 1


def float_to_fixed(x):
    return int(round(x * (1 << FRAC_BITS)))


def fixed_to_float(x):
    return float(x) / (1 << FRAC_BITS)


# -----------------------------------------------------------------------------
# FixedPointDivider: Q8.24-consistent LUT-seeded Newton-Raphson divider
# -----------------------------------------------------------------------------
def FixedPointDivider(clk, reset, dividend, divisor, quotient, start, done,
                      FRAC_BITS=FRAC_BITS, INT_BITS=INT_BITS, LUT_ADDR_BITS=8):
    WORD_WIDTH = INT_BITS + FRAC_BITS
    ROM_SIZE = 1 << LUT_ADDR_BITS
    ROM_MASK = ROM_SIZE - 1

    # Build ROM as a local tuple at elaboration time (safe for conversion)
    rom_temp = []
    for i in range(ROM_SIZE):
        v = 1.0 + (i + 0.5) * (1.0 / ROM_SIZE)  # values in [1.0, 2.0)
        seed_float = 1.0 / v
        rom_temp.append(int(round(seed_float * (1 << FRAC_BITS))))
    ROM_LOCAL = tuple(rom_temp)

    # FSM states
    IDLE, CAPTURE, NORMALIZE, LOOKUP, MUL1, MUL2, WRITE, DONE = range(8)
    state = Signal(intbv(IDLE, min=0, max=9))

    # ROM lookup signals
    rom_out = Signal(intbv(0, min=0, max=2**(FRAC_BITS+1)))
    rom_addr = Signal(intbv(0, min=0, max=ROM_SIZE))

    # Internal registers (typed)
    reg_dividend = Signal(intbv(0, min=-(2**(WORD_WIDTH+16)), max=2**(WORD_WIDTH+16)))
    reg_divisor  = Signal(intbv(0, min=-(2**(WORD_WIDTH+16)), max=2**(WORD_WIDTH+16)))
    reg_shift    = Signal(intbv(0, min=-(WORD_WIDTH+16), max=(WORD_WIDTH+16)))
    reg_seed     = Signal(intbv(0, min=0, max=2**(FRAC_BITS+16)))
    reg_index    = Signal(intbv(0, min=0, max=ROM_SIZE))
    reg_d_abs    = Signal(intbv(0, min=0, max=2**(WORD_WIDTH+16)))
    reg_d_norm   = Signal(intbv(0, min=0, max=2**(WORD_WIDTH+16)))
    mul1 = Signal(intbv(0, min=-(2**(2*WORD_WIDTH+64)), max=2**(2*WORD_WIDTH+64)))
    mul2 = Signal(intbv(0, min=-(2**(2*WORD_WIDTH+64)), max=2**(2*WORD_WIDTH+64)))
    done_reg = Signal(bool(0))

    # ROM lookup combinational (no chained comparison)
    @always_comb
    def rom_lookup():
        idx = int(rom_addr)
        if (idx >= 0) and (idx < ROM_SIZE):
            rom_out.next = ROM_LOCAL[idx]
        else:
            rom_out.next = ROM_LOCAL[0]

    @always_seq(clk.posedge, reset=reset)
    def fsm():
        done_reg.next = False

        if state == IDLE:
            done.next = False
            if start:
                reg_dividend.next = int(dividend)
                reg_divisor.next  = int(divisor)
                state.next = CAPTURE
            else:
                state.next = IDLE

        elif state == CAPTURE:
            d = int(reg_divisor)
            if d == 0:
                quotient.next = 0
                done_reg.next = True
                state.next = DONE
            else:
                if d >= 0:
                    reg_d_abs.next = d
                else:
                    reg_d_abs.next = -d
                state.next = NORMALIZE

        elif state == NORMALIZE:
            d_abs = int(reg_d_abs)
            msb_pos_val = 0
            # find msb position with a fixed loop range (conversion-safe)
            for bi in range(WORD_WIDTH + 16):
                b = (WORD_WIDTH + 16 - 1) - bi
                if (d_abs & (1 << b)) != 0:
                    msb_pos_val = b
                    break
            shift = msb_pos_val - FRAC_BITS
            reg_shift.next = shift
            state.next = LOOKUP

        elif state == LOOKUP:
            d_abs = int(reg_d_abs)
            shift = int(reg_shift)
            d_norm_val = 0
            index_val = 0

            if shift >= 0:
                d_norm_val = d_abs >> shift
            else:
                d_norm_val = d_abs << (-shift)

            reg_d_norm.next = d_norm_val

            if FRAC_BITS >= 8:
                frac_mask = (1 << FRAC_BITS) - 1
                index_val = (d_norm_val & frac_mask) >> (FRAC_BITS - 8)
            else:
                index_val = (d_norm_val << (8 - FRAC_BITS)) & ROM_MASK

            reg_index.next = index_val
            rom_addr.next = index_val
            state.next = MUL1

        elif state == MUL1:
            seed_val = int(rom_out)
            reg_seed.next = seed_val
            d_norm = int(reg_d_norm)
            prod = d_norm * seed_val
            mul1.next = prod
            state.next = MUL2

        elif state == MUL2:
            prod_q = int(mul1) >> FRAC_BITS
            two_q = (2 << FRAC_BITS)
            two_minus = two_q - prod_q
            xnext = (int(reg_seed) * two_minus) >> FRAC_BITS
            mul2.next = xnext
            state.next = WRITE

        elif state == WRITE:
            recip_q = int(mul2)
            shift = int(reg_shift)

            # default init for MyHDL conversion
            recip_actual_val = 0

            if shift > 0:
                recip_actual_val = recip_q >> shift
            elif shift < 0:
                recip_actual_val = recip_q << (-shift)
            else:
                recip_actual_val = recip_q

            if int(reg_divisor) < 0:
                recip_actual_val = -recip_actual_val

            prod_q = (int(reg_dividend) * recip_actual_val) >> FRAC_BITS
            quotient.next = prod_q

            done_reg.next = True
            state.next = DONE

        elif state == DONE:
            done_reg.next = True
            if not start:
                state.next = IDLE

        else:
            state.next = IDLE

        done.next = done_reg

    return fsm, rom_lookup


# -----------------------------------------------------------------------------
# LU Controller (sequential Doolittle algorithm, strict conversion-safe)
# N is compile-time constant above
# -----------------------------------------------------------------------------
def LUController(clk, reset, matrix_A, matrix_L, matrix_U, start, done):
    # FSM states
    IDLE, INIT, U_INIT, U_ACC, L_INIT, L_ACC, L_DIV_START, L_DIV_WAIT, NEXT_K, FINISHED = range(10)
    state = Signal(intbv(IDLE, min=0, max=11))

    # indices
    k = Signal(intbv(0, min=0, max=N+1))
    i = Signal(intbv(0, min=0, max=N+1))
    j = Signal(intbv(0, min=0, max=N+1))
    m = Signal(intbv(0, min=0, max=N+1))

    accumulator = Signal(intbv(0, min=-(2**(WORD_WIDTH+8)), max=2**(WORD_WIDTH+8)))

    # Divider wires
    div_dividend = Signal(intbv(0, min=-(2**WORD_WIDTH), max=2**WORD_WIDTH))
    div_divisor  = Signal(intbv(0, min=-(2**WORD_WIDTH), max=2**WORD_WIDTH))
    div_quotient = Signal(intbv(0, min=-(2**WORD_WIDTH), max=2**WORD_WIDTH))
    div_start    = Signal(bool(0))
    div_done     = Signal(bool(0))

    divider = FixedPointDivider(clk, reset,
                                div_dividend, div_divisor, div_quotient,
                                div_start, div_done)

    @always_seq(clk.posedge, reset=reset)
    def fsm():
        div_start.next = False

        if state == IDLE:
            done.next = False
            if start:
                state.next = INIT

        elif state == INIT:
            for idx in range(N*N):
                row = idx // N
                col = idx % N
                if row == col:
                    matrix_L[idx].next = (1 << FRAC_BITS)
                else:
                    matrix_L[idx].next = 0
                matrix_U[idx].next = 0

            k.next = 0
            j.next = 0
            state.next = U_INIT

        # U[k,j]
        elif state == U_INIT:
            if int(j) >= N:
                i.next = int(k) + 1
                state.next = L_INIT
            elif int(j) < int(k):
                j.next = int(j) + 1
            else:
                accumulator.next = int(matrix_A[int(k)*N + int(j)])
                m.next = 0
                state.next = U_ACC

        elif state == U_ACC:
            if int(m) < int(k):
                prod = (int(matrix_L[int(k)*N + int(m)]) *
                        int(matrix_U[int(m)*N + int(j)])) >> FRAC_BITS
                accumulator.next = int(accumulator) - prod
                m.next = int(m) + 1
            else:
                matrix_U[int(k)*N + int(j)].next = int(accumulator)
                j.next = int(j) + 1
                state.next = U_INIT

        # L[i,k]
        elif state == L_INIT:
            if int(i) >= N:
                state.next = NEXT_K
            else:
                accumulator.next = int(matrix_A[int(i)*N + int(k)])
                m.next = 0
                state.next = L_ACC

        elif state == L_ACC:
            if int(m) < int(k):
                prod = (int(matrix_L[int(i)*N + int(m)]) *
                        int(matrix_U[int(m)*N + int(k)])) >> FRAC_BITS
                accumulator.next = int(accumulator) - prod
                m.next = int(m) + 1
            else:
                div_dividend.next = int(accumulator)
                div_divisor.next  = int(matrix_U[int(k)*N + int(k)])
                div_start.next = True
                state.next = L_DIV_WAIT

        elif state == L_DIV_WAIT:
            if div_done:
                matrix_L[int(i)*N + int(k)].next = int(div_quotient)
                i.next = int(i) + 1
                state.next = L_INIT

        elif state == NEXT_K:
            k.next = int(k) + 1
            if (int(k) + 1) >= N:
                state.next = FINISHED
            else:
                j.next = 0
                state.next = U_INIT

        elif state == FINISHED:
            done.next = True
            if not start:
                state.next = IDLE

        else:
            state.next = IDLE

    return fsm, divider


# -----------------------------------------------------------------------------
# Testbench (simulation only)
# -----------------------------------------------------------------------------
def testbench():
    import numpy as np
    np.random.seed(0)

    clk = Signal(bool(0))
    reset = ResetSignal(1, active=1, isasync=False)
    start = Signal(bool(0))
    done = Signal(bool(0))

    matrix_A = [Signal(intbv(0, min=-(2**WORD_WIDTH), max=2**WORD_WIDTH)) for _ in range(N*N)]
    matrix_L = [Signal(intbv(0, min=-(2**WORD_WIDTH), max=2**WORD_WIDTH)) for _ in range(N*N)]
    matrix_U = [Signal(intbv(0, min=-(2**WORD_WIDTH), max=2**WORD_WIDTH)) for _ in range(N*N)]

    dut = LUController(clk, reset, matrix_A, matrix_L, matrix_U, start, done)

    @instance
    def clkgen():
        while True:
            yield delay(5)
            clk.next = not clk

    @instance
    def stimulus():
        A = (np.random.rand(N, N) * 5.0) + 1.0
        print("Input A:")
        print(A)

        # Reference Doolittle (no pivot)
        L_ref = np.eye(N)
        U_ref = np.zeros((N, N))
        for k in range(N):
            for j in range(k, N):
                U_ref[k,j] = A[k,j] - np.sum(L_ref[k,:k] * U_ref[:k,j])
            for i in range(k+1, N):
                L_ref[i,k] = (A[i,k] - np.sum(L_ref[i,:k] * U_ref[:k,k])) / U_ref[k,k]

        print("Reference L:")
        print(L_ref)
        print("Reference U:")
        print(U_ref)

        # load A
        for i in range(N):
            for j in range(N):
                matrix_A[i*N+j].next = float_to_fixed(A[i,j])

        # reset
        reset.next = True
        yield clk.posedge
        reset.next = False
        yield clk.posedge

        # start
        start.next = True
        yield clk.posedge
        start.next = False

        cycles = 0
        while not done:
            yield clk.posedge
            cycles += 1
            if cycles > 100000:
                print("Timeout!")
                break

        print("Completed in", cycles, "cycles")

        L_hw = [[fixed_to_float(int(matrix_L[i*N+j])) for j in range(N)] for i in range(N)]
        U_hw = [[fixed_to_float(int(matrix_U[i*N+j])) for j in range(N)] for i in range(N)]

        print("Hardware L:")
        for row in L_hw: print(row)

        print("Hardware U:")
        for row in U_hw: print(row)

        raise StopSimulation

    return dut, clkgen, stimulus


# -----------------------------------------------------------------------------
# Top-level for VHDL generation or simulation
# -----------------------------------------------------------------------------
# Create a zero-argument top() that instantiates DUT and returns the generators
# This is the recommended (non-deprecated) way to call toVHDL per MEP-114.
def top(clk, reset, start, done, matrix_A_in, matrix_L_out, matrix_U_out):

# Internal lists of scalar Signals for LUController
    A_list = [Signal(intbv(0, min=-(1 << (ELEMENT_BITS-1)), max=(1 << (ELEMENT_BITS-1)))) for _ in range(NUM_ELEM)]
    L_list = [Signal(intbv(0, min=-(1 << (ELEMENT_BITS-1)), max=(1 << (ELEMENT_BITS-1)))) for _ in range(NUM_ELEM)]
    U_list = [Signal(intbv(0, min=-(1 << (ELEMENT_BITS-1)), max=(1 << (ELEMENT_BITS-1)))) for _ in range(NUM_ELEM)]


# Instantiate the existing LUController which operates on lists
    lu_inst = LUController(clk, reset, A_list, L_list, U_list, start, done)


# combinational mapping: flat input vector -> A_list elements
    @always_comb
    def unflatten_inputs():
# read flat integer value
        for i in range(NUM_ELEM):
    # row-major: element i occupies bits [i*32 + 31 : i*32]
            shift = i * ELEMENT_BITS
            temp = matrix_A_in[shift+ELEMENT_BITS:shift]
        # interpret as signed 32-bit
            if temp[ELEMENT_BITS-1] == 1:
                A_list[i].next = int(temp) - (1 << ELEMENT_BITS)
            else:
                A_list[i].next = int(temp)


# combinational mapping: L_list/U_list -> flat output vectors
    @always_comb
    def flatten_outputs():
        outL = 0
        outU = 0
    # pack in row-major, element 0 at LSB
        for i in range(NUM_ELEM):
           lv = int(L_list[i]) & MASK_32
           uv = int(U_list[i]) & MASK_32
           outL = outL | (lv << (i * ELEMENT_BITS))
           outU = outU | (uv << (i * ELEMENT_BITS))
        matrix_L_out.next = intbv(outL)[FLAT_WIDTH:]
        matrix_U_out.next = intbv(outU)[FLAT_WIDTH:]

    return lu_inst, unflatten_inputs, flatten_outputs

if __name__ == "__main__":
    import sys
    import os

    if len(sys.argv) > 1 and sys.argv[1] == "--vhdl":
        print("Converting to VHDL... (using top() per MEP-114)")

        if not os.path.exists("vhdl_out"):
            os.makedirs("vhdl_out")

        toVHDL.directory = "vhdl_out"
        # call toVHDL with the zero-argument top function
        clk = Signal(bool(0))
        reset = ResetSignal(0, active=1, isasync=False)
        start = Signal(bool(0))
        done = Signal(bool(0))

        matrix_A_in  = Signal(intbv(0)[32*16:])
        matrix_L_out = Signal(intbv(0)[32*16:])
        matrix_U_out = Signal(intbv(0)[32*16:])

        toVHDL(top,
               clk,
               reset,
               start,
               done,
               matrix_A_in,
               matrix_L_out,
               matrix_U_out)
        print("VHDL generated in vhdl_out/")
    else:
        print("Running simulation...")
        sim = Simulation(*testbench())
        sim.run()
