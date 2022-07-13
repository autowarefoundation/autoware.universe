# ifndef CPPAD_LOCAL_PLAY_PLAYER_HPP
# define CPPAD_LOCAL_PLAY_PLAYER_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/play/addr_enum.hpp>
# include <cppad/local/play/sequential_iterator.hpp>
# include <cppad/local/play/subgraph_iterator.hpp>
# include <cppad/local/play/random_setup.hpp>
# include <cppad/local/atom_state.hpp>
# include <cppad/local/is_pod.hpp>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file player.hpp
File used to define the player class.
*/

/*!
Class used to store and play back an operation sequence recording.

\tparam Base
These were AD< Base > operations when recorded. Operations during playback
are done using the type Base .
*/

template <class Base>
class player {
    // player<Base> must be a friend of player< AD<Base> > for base2ad to work
    template <class AnotherBase> friend class player;
private:
    // ----------------------------------------------------------------------
    // information that defines the recording

    /// Number of independent dynamic parameters
    size_t num_dynamic_ind_;

    /// Number of variables in the recording.
    size_t num_var_rec_;

    /// number of vecad load opeations in the reconding
    size_t num_var_load_rec_;

    /// Number of VecAD vectors in the recording
    size_t num_var_vecad_rec_;

    /// The operators in the recording.
    pod_vector<opcode_t> op_vec_;

    /// The operation argument indices in the recording
    pod_vector<addr_t> arg_vec_;

    /// Character strings ('\\0' terminated) in the recording.
    pod_vector<char> text_vec_;

    /// The VecAD indices in the recording.
    pod_vector<addr_t> all_var_vecad_ind_;

    /// All of the parameters in the recording.
    /// Use pod_maybe because Base may not be plain old data.
    pod_vector_maybe<Base> all_par_vec_;

    /// Which elements of all_par_vec_ are dynamic parameters
    /// (size equal number of parametrers)
    pod_vector<bool> dyn_par_is_;

    /// mapping from dynamic parameter index to parameter index
    /// 1: size equal to number of dynamic parameters
    /// 2: dyn_ind2par_ind_[j] < dyn_ind2par_ind_[j+1]
    pod_vector<addr_t> dyn_ind2par_ind_;

    /// operators for just the dynamic parameters
    /// (size equal number of dynamic parameters)
    pod_vector<opcode_t> dyn_par_op_;

    /// arguments for the dynamic parameter operators
    pod_vector<addr_t> dyn_par_arg_;

    // ----------------------------------------------------------------------
    // Information needed to use member functions that begin with random_
    // and for using const_subgraph_iterator.

    /// index in arg_vec_ corresonding to the first argument for each operator
    pod_vector<unsigned char> op2arg_vec_;

    /*!
    Index of the result variable for each operator. If the operator has
    no results, this is not defined. The invalid index num_var_rec_ is used
    when NDEBUG is not defined. If the operator has more than one result, this
    is the primary result; i.e., the last result. Auxillary are only used by
    the operator and not used by other operators.
    */
    pod_vector<unsigned char> op2var_vec_;

    /// Mapping from primary variable index to corresponding operator index.
    /// This is used to traverse sub-graphs of the operation sequence.
    /// This value is valid (invalid) for primary (auxillary) variables.
    pod_vector<unsigned char> var2op_vec_;

public:
    // =================================================================
    /// default constructor
    // set all scalars to zero to avoid valgraind warning when ani assignment
    // occures before values get set.
    player(void) :
    num_dynamic_ind_(0)  ,
    num_var_rec_(0)      ,
    num_var_load_rec_(0)  ,
    num_var_vecad_rec_(0)
    { }
    // move semantics constructor
    // (none of the default constructor values matter to the destructor)
    player(player& play)
    {   swap(play);  }
    // =================================================================
    /// destructor
    ~player(void)
    { }
    // ======================================================================
    /// type used for addressing iterators for this player
    play::addr_enum address_type(void) const
    {
        // required
        size_t required = 0;
        required = std::max(required, num_var_rec_   );  // number variables
        required = std::max(required, op_vec_.size()  ); // number operators
        required = std::max(required, arg_vec_.size() ); // number arguments
        //
        // unsigned short
        if( required <= std::numeric_limits<unsigned short>::max() )
            return play::unsigned_short_enum;
        //
        // unsigned int
        if( required <= std::numeric_limits<unsigned int>::max() )
            return play::unsigned_int_enum;
        //
        // unsigned size_t
        CPPAD_ASSERT_UNKNOWN(
            required <= std::numeric_limits<size_t>::max()
        );
        return play::size_t_enum;
    }
    // ===============================================================
    /*!
    Moving an operation sequence from a recorder to this player

    \param rec
    the object that was used to record the operation sequence. After this
    operation, the state of the recording is no longer defined. For example,
    the pod_vector member variables in this have been swapped with rec.

    \param n_ind
    the number of independent variables (only used for error checking
    when NDEBUG is not defined).

    \par
    Use an assert to check that the length of the following vectors is
    less than the maximum possible value for addr_t; i.e., that an index
    in these vectors can be represented using the type addr_t:
    op_vec_, all_var_vecad_ind_, arg_vec_, test_vec_, all_par_vec_, text_vec_,
    dyn_par_arg_.
    */
    void get_recording(recorder<Base>& rec, size_t n_ind)
    {
# ifndef NDEBUG
        size_t addr_t_max = size_t( std::numeric_limits<addr_t>::max() );
# endif
        // just set size_t values
        num_dynamic_ind_    = rec.num_dynamic_ind_;
        num_var_rec_        = rec.num_var_rec_;
        num_var_load_rec_   = rec.num_var_load_rec_;

        // op_vec_
        op_vec_.swap(rec.op_vec_);
        CPPAD_ASSERT_UNKNOWN(op_vec_.size() < addr_t_max );

        // op_arg_vec_
        arg_vec_.swap(rec.arg_vec_);
        CPPAD_ASSERT_UNKNOWN(arg_vec_.size()    < addr_t_max );

        // all_par_vec_
        all_par_vec_.swap(rec.all_par_vec_);
        CPPAD_ASSERT_UNKNOWN(all_par_vec_.size() < addr_t_max );

        // dyn_par_is_, dyn_par_op_, dyn_par_arg_
        dyn_par_is_.swap( rec.dyn_par_is_ );
        dyn_par_op_.swap( rec.dyn_par_op_ );
        dyn_par_arg_.swap( rec.dyn_par_arg_ );
        CPPAD_ASSERT_UNKNOWN(dyn_par_arg_.size() < addr_t_max );

        // text_rec_
        text_vec_.swap(rec.text_vec_);
        CPPAD_ASSERT_UNKNOWN(text_vec_.size() < addr_t_max );

        // all_var_vecad_ind_
        all_var_vecad_ind_.swap(rec.all_var_vecad_ind_);
        CPPAD_ASSERT_UNKNOWN(all_var_vecad_ind_.size() < addr_t_max );

        // num_var_vecad_rec_
        num_var_vecad_rec_ = 0;
        {   // all_var_vecad_ind_ contains size of each VecAD followed by
            // the parameter indices used to inialize it.
            size_t i = 0;
            while( i < all_var_vecad_ind_.size() )
            {   num_var_vecad_rec_++;
                i += size_t( all_var_vecad_ind_[i] ) + 1;
            }
            CPPAD_ASSERT_UNKNOWN( i == all_var_vecad_ind_.size() );
        }

        // mapping from dynamic parameter index to parameter index
        dyn_ind2par_ind_.resize( dyn_par_op_.size() );
        size_t i_dyn = 0;
        for(size_t i_par = 0; i_par < all_par_vec_.size(); ++i_par)
        {   if( dyn_par_is_[i_par] )
            {   dyn_ind2par_ind_[i_dyn] = addr_t( i_par );
                ++i_dyn;
            }
        }
        CPPAD_ASSERT_UNKNOWN( i_dyn == dyn_ind2par_ind_.size() );

        // random access information
        clear_random();

        // some checks
        check_inv_op(n_ind);
        check_variable_dag();
        check_dynamic_dag();
    }
    // ----------------------------------------------------------------------
    /*!
    Check that InvOp operators start with second operator and are contiguous,
    and there are n_ind of them.
    */
# ifdef NDEBUG
    void check_inv_op(size_t n_ind) const
    {   return; }
# else
    void check_inv_op(size_t n_ind) const
    {   play::const_sequential_iterator itr = begin();
        OpCode        op;
        const addr_t* op_arg;
        size_t        var_index;
        itr.op_info(op, op_arg, var_index);
        CPPAD_ASSERT_UNKNOWN( op == BeginOp );
        size_t i_op = 0;
        while( op != EndOp )
        {   // start at second operator
            (++itr).op_info(op, op_arg, var_index);
            ++i_op;
            CPPAD_ASSERT_UNKNOWN( (op == InvOp) == (i_op <= n_ind) );
        }
        return;
    }
# endif
    // ----------------------------------------------------------------------
    /*!
    Check variable graph to make sure arguments have value less
    than or equal to the previously created variable. This is the directed
    acyclic graph condition (DAG).
    */
# ifdef NDEBUG
    void check_variable_dag(void) const
    {   return; }
# else
    void check_variable_dag(void) const
    {   play::const_sequential_iterator itr = begin();
        OpCode        op;
        const addr_t* op_arg;
        size_t        var_index;
        itr.op_info(op, op_arg, var_index);
        CPPAD_ASSERT_UNKNOWN( op == BeginOp );
        //
        addr_t arg_var_bound = 0;
        while( op != EndOp )
        {   (++itr).op_info(op, op_arg, var_index);
            switch(op)
            {
                // cases where nothing to do
                case BeginOp:
                case EndOp:
                case EqppOp:
                case InvOp:
                case LdpOp:
                case LeppOp:
                case LtppOp:
                case NeppOp:
                case ParOp:
                case AFunOp:
                case FunapOp:
                case FunrpOp:
                case FunrvOp:
                case StppOp:
                break;

                // only first argument is a variable
                case AbsOp:
                case AcosOp:
                case AcoshOp:
                case AsinOp:
                case AsinhOp:
                case AtanOp:
                case AtanhOp:
                case CosOp:
                case CoshOp:
                case DivvpOp:
                case ErfOp:
                case ErfcOp:
                case ExpOp:
                case Expm1Op:
                case LevpOp:
                case LogOp:
                case Log1pOp:
                case LtvpOp:
                case NegOp:
                case PowvpOp:
                case SignOp:
                case SinOp:
                case SinhOp:
                case SqrtOp:
                case SubvpOp:
                case TanOp:
                case TanhOp:
                case FunavOp:
                case ZmulvpOp:
                CPPAD_ASSERT_UNKNOWN(op_arg[0] <= arg_var_bound );
                break;

                // only second argument is a variable
                case AddpvOp:
                case DisOp:
                case DivpvOp:
                case EqpvOp:
                case LdvOp:
                case LepvOp:
                case LtpvOp:
                case MulpvOp:
                case NepvOp:
                case PowpvOp:
                case StvpOp:
                case SubpvOp:
                case ZmulpvOp:
                CPPAD_ASSERT_UNKNOWN(op_arg[1] <= arg_var_bound );
                break;

                // only first and second arguments are variables
                case AddvvOp:
                case DivvvOp:
                case EqvvOp:
                case LevvOp:
                case LtvvOp:
                case MulvvOp:
                case NevvOp:
                case PowvvOp:
                case SubvvOp:
                case ZmulvvOp:
                CPPAD_ASSERT_UNKNOWN(op_arg[0] <= arg_var_bound );
                CPPAD_ASSERT_UNKNOWN(op_arg[1] <= arg_var_bound );
                break;

                // StpvOp
                case StpvOp:
                CPPAD_ASSERT_UNKNOWN(op_arg[2] <= arg_var_bound );
                break;

                // StvvOp
                case StvvOp:
                CPPAD_ASSERT_UNKNOWN(op_arg[1] <= arg_var_bound );
                CPPAD_ASSERT_UNKNOWN(op_arg[2] <= arg_var_bound );
                break;

                // CSumOp
                case CSumOp:
                {   CPPAD_ASSERT_UNKNOWN( 5 < op_arg[2] );
                    for(addr_t j = 5; j < op_arg[2]; j++)
                        CPPAD_ASSERT_UNKNOWN(op_arg[j] <= arg_var_bound);
                }
                itr.correct_before_increment();
                break;

                // CExpOp
                case CExpOp:
                if( op_arg[1] & 1 )
                    CPPAD_ASSERT_UNKNOWN( op_arg[2] <= arg_var_bound);
                if( op_arg[1] & 2 )
                    CPPAD_ASSERT_UNKNOWN( op_arg[3] <= arg_var_bound);
                if( op_arg[1] & 4 )
                    CPPAD_ASSERT_UNKNOWN( op_arg[4] <= arg_var_bound);
                if( op_arg[1] & 8 )
                    CPPAD_ASSERT_UNKNOWN( op_arg[5] <= arg_var_bound);
                break;

                // PriOp
                case PriOp:
                if( op_arg[0] & 1 )
                    CPPAD_ASSERT_UNKNOWN( op_arg[1] <= arg_var_bound);
                if( op_arg[0] & 2 )
                    CPPAD_ASSERT_UNKNOWN( op_arg[3] <= arg_var_bound);
                break;

                // CSkipOp
                case CSkipOp:
                if( op_arg[1] & 1 )
                    CPPAD_ASSERT_UNKNOWN( op_arg[2] <= arg_var_bound);
                if( op_arg[1] & 2 )
                    CPPAD_ASSERT_UNKNOWN( op_arg[3] <= arg_var_bound);
                itr.correct_before_increment();
                break;

                default:
                CPPAD_ASSERT_UNKNOWN(false);
                break;


            }
            if( NumRes(op) > 0 )
            {   if( var_index > 0 )
                {   CPPAD_ASSERT_UNKNOWN(size_t(arg_var_bound) < var_index);
                }
                else
                {   CPPAD_ASSERT_UNKNOWN(size_t(arg_var_bound) == var_index);
                }
                //
                arg_var_bound = addr_t(var_index);
            }
        }
        return;
    }
# endif
    // ----------------------------------------------------------------------
    /*!
    Check dynamic parameter graph to make sure arguments have value less
    than or equal to the previously created dynamic parameter.
    This is the directed acyclic graph condition (DAG).
    */
# ifdef NDEBUG
    void check_dynamic_dag(void) const
    {   return; }
# else
    void check_dynamic_dag(void) const
    {   // number of dynamic parameters
        size_t num_dyn = dyn_par_op_.size();
        //
        size_t i_arg = 0; // initialize dynamic parameter argument index
        for(size_t i_dyn = 0; i_dyn < num_dyn; ++i_dyn)
        {   // i_par is parameter index
            addr_t i_par = dyn_ind2par_ind_[i_dyn];
            CPPAD_ASSERT_UNKNOWN( dyn_par_is_[i_par] );
            //
            // operator for this dynamic parameter
            op_code_dyn op = op_code_dyn( dyn_par_op_[i_dyn] );
            //
            // number of arguments for this dynamic parameter
            size_t n_arg       = num_arg_dyn(op);
            if( op == atom_dyn )
            {   size_t n = size_t( dyn_par_arg_[i_arg + 1] );
                size_t m = size_t( dyn_par_arg_[i_arg + 2] );
                n_arg    = 5 + n + m;
                CPPAD_ASSERT_UNKNOWN(
                    n_arg == size_t( dyn_par_arg_[i_arg + 4 + n + m] )
                );
                for(size_t i = 4; i < n - 1; ++i)
                    CPPAD_ASSERT_UNKNOWN( dyn_par_arg_[i_arg + i] <  i_par );
# ifndef NDEBUG
                for(size_t i = 4+n; i < 4+n+m; ++i)
                {   addr_t j_par = dyn_par_arg_[i_arg + i];
                    CPPAD_ASSERT_UNKNOWN( (j_par == 0) || (j_par >= i_par) );
                }
# endif
            }
            else
            {   size_t num_non_par = num_non_par_arg_dyn(op);
                for(size_t i = num_non_par; i < n_arg; ++i)
                    CPPAD_ASSERT_UNKNOWN( dyn_par_arg_[i_arg + i] < i_par);
            }
            //
            // next dynamic parameter
            i_arg += n_arg;
        }
        return;
    }
# endif
    // ===============================================================
    /*!
    Copy a player<Base> to another player<Base>

    \param play
    object that contains the operatoion sequence to copy.
    */
    void operator=(const player& play)
    {
        // size_t objects
        num_dynamic_ind_    = play.num_dynamic_ind_;
        num_var_rec_        = play.num_var_rec_;
        num_var_load_rec_   = play.num_var_load_rec_;
        num_var_vecad_rec_  = play.num_var_vecad_rec_;
        //
        // pod_vectors
        op_vec_             = play.op_vec_;
        arg_vec_            = play.arg_vec_;
        text_vec_           = play.text_vec_;
        all_var_vecad_ind_  = play.all_var_vecad_ind_;
        dyn_par_is_         = play.dyn_par_is_;
        dyn_ind2par_ind_    = play.dyn_ind2par_ind_;
        dyn_par_op_         = play.dyn_par_op_;
        dyn_par_arg_        = play.dyn_par_arg_;
        op2arg_vec_         = play.op2arg_vec_;
        op2var_vec_         = play.op2var_vec_;
        var2op_vec_         = play.var2op_vec_;
        //
        // pod_maybe_vectors
        all_par_vec_        = play.all_par_vec_;
    }
    // ===============================================================
    /// Create a player< AD<Base> > from this player<Base>
    player< AD<Base> > base2ad(void) const
    {   player< AD<Base> > play;
        //
        // size_t objects
        play.num_dynamic_ind_    = num_dynamic_ind_;
        play.num_var_rec_        = num_var_rec_;
        play.num_var_load_rec_   = num_var_load_rec_;
        play.num_var_vecad_rec_  = num_var_vecad_rec_;
        //
        // pod_vectors
        play.op_vec_             = op_vec_;
        play.arg_vec_            = arg_vec_;
        play.text_vec_           = text_vec_;
        play.all_var_vecad_ind_  = all_var_vecad_ind_;
        play.dyn_par_is_         = dyn_par_is_;
        play.dyn_ind2par_ind_    = dyn_ind2par_ind_;
        play.dyn_par_op_         = dyn_par_op_;
        play.dyn_par_arg_        = dyn_par_arg_;
        play.op2arg_vec_         = op2arg_vec_;
        play.op2var_vec_         = op2var_vec_;
        play.var2op_vec_         = var2op_vec_;
        //
        // pod_maybe_vector< AD<Base> > = pod_maybe_vector<Base>
        play.all_par_vec_.resize( all_par_vec_.size() );
        for(size_t i = 0; i < all_par_vec_.size(); ++i)
            play.all_par_vec_[i] = all_par_vec_[i];
        //
        return play;
    }
    // ===============================================================
    /// swap this recording with another recording
    /// (used for move semantics version of ADFun assignment operation)
    void swap(player& other)
    {   // size_t objects
        std::swap(num_dynamic_ind_,    other.num_dynamic_ind_);
        std::swap(num_var_rec_,        other.num_var_rec_);
        std::swap(num_var_load_rec_,   other.num_var_load_rec_);
        std::swap(num_var_vecad_rec_,  other.num_var_vecad_rec_);
        //
        // pod_vectors
        op_vec_.swap(             other.op_vec_);
        arg_vec_.swap(            other.arg_vec_);
        text_vec_.swap(           other.text_vec_);
        all_var_vecad_ind_.swap(  other.all_var_vecad_ind_);
        dyn_par_is_.swap(         other.dyn_par_is_);
        dyn_ind2par_ind_.swap(    other.dyn_ind2par_ind_);
        dyn_par_op_.swap(         other.dyn_par_op_);
        dyn_par_arg_.swap(        other.dyn_par_arg_);
        op2arg_vec_.swap(         other.op2arg_vec_);
        op2var_vec_.swap(         other.op2var_vec_);
        var2op_vec_.swap(         other.var2op_vec_);
        //
        // pod_maybe_vectors
        all_par_vec_.swap(    other.all_par_vec_);
    }
    // move semantics assignment
    void operator=(player&& play)
    {   swap(play); }
    // =================================================================
    /// Enable use of const_subgraph_iterator and member functions that begin
    // with random_(no work if already setup).
    template <class Addr>
    void setup_random(void)
    {   play::random_setup(
            num_var_rec_                               ,
            op_vec_                                    ,
            arg_vec_                                   ,
            op2arg_vec_.pod_vector_ptr<Addr>()         ,
            op2var_vec_.pod_vector_ptr<Addr>()         ,
            var2op_vec_.pod_vector_ptr<Addr>()
        );
    }
    /// Free memory used for functions that begin with random_
    /// and random iterators and subgraph iterators
    void clear_random(void)
    {
        op2arg_vec_.clear();
        op2var_vec_.clear();
        var2op_vec_.clear();
        CPPAD_ASSERT_UNKNOWN( op2arg_vec_.size() == 0  );
        CPPAD_ASSERT_UNKNOWN( op2var_vec_.size() == 0  );
        CPPAD_ASSERT_UNKNOWN( var2op_vec_.size() == 0  );
    }
    /// get non-const version of all_par_vec
    pod_vector_maybe<Base>& all_par_vec(void)
    {   return all_par_vec_; }
    /// get non-const version of all_par_vec
    const pod_vector_maybe<Base>& all_par_vec(void) const
    {   return all_par_vec_; }
    // ================================================================
    // const functions that retrieve infromation from this player
    // ================================================================
    /// const version of dynamic parameter flag
    const pod_vector<bool>& dyn_par_is(void) const
    {   return dyn_par_is_; }
    /// const version of dynamic parameter index to parameter index
    const pod_vector<addr_t>& dyn_ind2par_ind(void) const
    {   return dyn_ind2par_ind_; }
    /// const version of dynamic parameter operator
    const pod_vector<opcode_t>& dyn_par_op(void) const
    {   return dyn_par_op_; }
    /// const version of dynamic parameter arguments
    const pod_vector<addr_t>& dyn_par_arg(void) const
    {   return dyn_par_arg_; }
    /*!
    \brief
    fetch an operator from the recording.

    \return
    the i-th operator in the recording.

    \param i
    the index of the operator in recording
    */
    OpCode GetOp (size_t i) const
    {   return OpCode(op_vec_[i]); }

    /*!
    \brief
    Fetch a VecAD index from the recording.

    \return
    the i-th VecAD index in the recording.

    \param i
    the index of the VecAD index in recording
    */
    size_t GetVecInd (size_t i) const
    {   return size_t( all_var_vecad_ind_[i] ); }

    /*!
    \brief
    Fetch a parameter from the recording.

    \return
    the i-th parameter in the recording.

    \param i
    the index of the parameter in recording
    */
    Base GetPar(size_t i) const
    {   return all_par_vec_[i]; }

    /*!
    \brief
    Fetch entire parameter vector from the recording.

    \return
    the entire parameter vector.

    */
    const Base* GetPar(void) const
    {   return all_par_vec_.data(); }

    /*!
    \brief
    Fetch a '\\0' terminated string from the recording.

    \return
    the beginning of the string.

    \param i
    the index where the string begins.
    */
    const char *GetTxt(size_t i) const
    {   CPPAD_ASSERT_UNKNOWN(i < text_vec_.size() );
        return text_vec_.data() + i;
    }

    /// Fetch number of independent dynamic parameters in the recording
    size_t num_dynamic_ind(void) const
    {   return num_dynamic_ind_; }

    /// Fetch number of dynamic parameters in the recording
    size_t num_dynamic_par(void) const
    {   return dyn_par_op_.size(); }

    /// Fetch number of dynamic parameters operator arguments in the recording
    size_t num_dynamic_arg(void) const
    {   return dyn_par_arg_.size(); }

    /// Fetch number of variables in the recording.
    size_t num_var_rec(void) const
    {   return num_var_rec_; }

    /// Fetch number of vecad load operations
    size_t num_var_load_rec(void) const
    {   return num_var_load_rec_; }

    /// Fetch number of operators in the recording.
    size_t num_op_rec(void) const
    {   return op_vec_.size(); }

    /// Fetch number of VecAD indices in the recording.
    size_t num_var_vecad_ind_rec(void) const
    {   return all_var_vecad_ind_.size(); }

    /// Fetch number of VecAD vectors in the recording
    size_t num_var_vecad_rec(void) const
    {   return num_var_vecad_rec_; }

    /// Fetch number of argument indices in the recording.
    size_t num_op_arg_rec(void) const
    {   return arg_vec_.size(); }

    /// Fetch number of parameters in the recording.
    size_t num_par_rec(void) const
    {   return all_par_vec_.size(); }

    /// Fetch number of characters (representing strings) in the recording.
    size_t num_text_rec(void) const
    {   return text_vec_.size(); }

    /// A measure of amount of memory used to store
    /// the operation sequence, just lengths, not capacities.
    /// In user api as f.size_op_seq(); see the file fun_property.omh.
    size_t size_op_seq(void) const
    {   // check assumptions made by ad_fun<Base>::size_op_seq()
        CPPAD_ASSERT_UNKNOWN( op_vec_.size() == num_op_rec() );
        CPPAD_ASSERT_UNKNOWN( arg_vec_.size()    == num_op_arg_rec() );
        CPPAD_ASSERT_UNKNOWN( all_par_vec_.size() == num_par_rec() );
        CPPAD_ASSERT_UNKNOWN( text_vec_.size() == num_text_rec() );
        CPPAD_ASSERT_UNKNOWN( all_var_vecad_ind_.size() == num_var_vecad_ind_rec() );
        return op_vec_.size()        * sizeof(opcode_t)
             + arg_vec_.size()       * sizeof(addr_t)
             + all_par_vec_.size()   * sizeof(Base)
             + dyn_par_is_.size()    * sizeof(bool)
             + dyn_ind2par_ind_.size() * sizeof(addr_t)
             + dyn_par_op_.size()    * sizeof(opcode_t)
             + dyn_par_arg_.size()   * sizeof(addr_t)
             + text_vec_.size()      * sizeof(char)
             + all_var_vecad_ind_.size() * sizeof(addr_t)
        ;
    }
    /// A measure of amount of memory used for random access routine
    /// In user api as f.size_random(); see the file fun_property.omh.
    size_t size_random(void) const
    {
# ifndef NDEBUG
        if( op2arg_vec_.size() == 0 )
        {   CPPAD_ASSERT_UNKNOWN( op2var_vec_.size() == 0 );
            CPPAD_ASSERT_UNKNOWN( var2op_vec_.size() == 0 );
        }
        else
        {   size_t size = 0;
            switch( address_type() )
            {   case play::unsigned_short_enum:
                size = sizeof(unsigned short);
                break;
                //
                case play::unsigned_int_enum:
                size = sizeof(unsigned int);
                break;
                //
                case play::size_t_enum:
                size = sizeof(size_t);
                break;

                default:
                CPPAD_ASSERT_UNKNOWN(false);
                break;
            }
            CPPAD_ASSERT_UNKNOWN( op2arg_vec_.size()/size  == num_op_rec() );
            CPPAD_ASSERT_UNKNOWN( op2var_vec_.size()/size  == num_op_rec() );
            CPPAD_ASSERT_UNKNOWN( var2op_vec_.size()/size  == num_var_rec() );
        }
# endif
        CPPAD_ASSERT_UNKNOWN( sizeof(unsigned char) == 1 );
        return op2arg_vec_.size()
             + op2var_vec_.size()
             + var2op_vec_.size()
        ;
    }
    // -----------------------------------------------------------------------
    /// const sequential iterator begin
    play::const_sequential_iterator begin(void) const
    {   size_t op_index = 0;
        size_t num_var  = num_var_rec_;
        return play::const_sequential_iterator(
            num_var, &op_vec_, &arg_vec_, op_index
        );
    }
    /// const sequential iterator end
    play::const_sequential_iterator end(void) const
    {   size_t op_index = op_vec_.size() - 1;
        size_t num_var  = num_var_rec_;
        return play::const_sequential_iterator(
            num_var, &op_vec_, &arg_vec_, op_index
        );
    }
    // -----------------------------------------------------------------------
    /// const subgraph iterator begin
    play::const_subgraph_iterator<addr_t>  begin_subgraph(
        const play::const_random_iterator<addr_t>& random_itr ,
        const pod_vector<addr_t>*                  subgraph   ) const
    {   size_t subgraph_index = 0;
        return play::const_subgraph_iterator<addr_t>(
            random_itr,
            subgraph,
            subgraph_index
        );
    }
    /// const subgraph iterator end
    template <class Addr>
    play::const_subgraph_iterator<Addr>  end_subgraph(
        const play::const_random_iterator<Addr>&   random_itr ,
        const pod_vector<addr_t>*                  subgraph   ) const
    {   size_t subgraph_index = subgraph->size() - 1;
        return play::const_subgraph_iterator<Addr>(
            random_itr,
            subgraph,
            subgraph_index
        );
    }
    // -----------------------------------------------------------------------
    /// const random iterator
    template <class Addr>
    play::const_random_iterator<Addr> get_random(void) const
    {   return play::const_random_iterator<Addr>(
            op_vec_,
            arg_vec_,
            op2arg_vec_.pod_vector_ptr<Addr>(),
            op2var_vec_.pod_vector_ptr<Addr>(),
            var2op_vec_.pod_vector_ptr<Addr>()
        );
    }
};

} } // END_CPPAD_lOCAL_NAMESPACE
# endif
