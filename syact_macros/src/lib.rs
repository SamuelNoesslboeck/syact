#![crate_name = "syact_macros"]
#![doc = include_str!("../README.md")]
#![deny(missing_docs)]

use core::str::FromStr;

use proc_macro2::TokenStream;
use syn::DeriveInput;

// SyncActuatorGroup
    fn sync_comp_group_impl(ast : DeriveInput, comp_type : TokenStream) -> proc_macro::TokenStream {
        match ast.data {
            // Macro works only on structs
            syn::Data::Struct(data) => {
                // Struct metadata
                let name = ast.ident;
                let fields = data.fields;
                let fields_count = fields.len();

                // Streams for different functions
                let mut setup_s = TokenStream::new();
                let mut for_each_s = TokenStream::new();
                let mut for_each_mut_s = TokenStream::new();
                let mut try_for_each_s = TokenStream::new();
                let mut try_for_each_mut_s = TokenStream::new();

                let mut findex : usize  = 0;
                for field in fields {
                    // Name of current field
                    let fname = if let Some(ident) = field.ident {
                        quote::quote! { #ident }
                    } else {
                        TokenStream::from_str(&format!("{}", findex)).unwrap()
                    };  

                    // Extend streams
                        setup_s.extend(quote::quote! {
                            self.#fname.setup()?;
                        });

                        for_each_s.extend(quote::quote! {
                            res[#findex] = func(&self.#fname, #findex);
                        }); 

                        for_each_mut_s.extend(quote::quote! {
                            res[#findex] = func(&mut self.#fname, #findex);
                        }); 

                        try_for_each_s.extend(quote::quote! {
                            res[#findex] = func(&self.#fname, #findex)?;
                        }); 

                        try_for_each_mut_s.extend(quote::quote! {
                            res[#findex] = func(&mut self.#fname, #findex)?;
                        }); 
                    // 

                    findex += 1;
                }

                quote::quote! {
                    impl syact::act::group::SyncActuatorGroup<#comp_type, #fields_count> for #name { 
                        fn for_each<'a, F, R>(&'a self, mut func : F) -> [R; #fields_count]
                        where 
                            F : FnMut(&'a #comp_type, usize) -> R
                        {   
                            let mut res = unsafe { core::mem::zeroed::<[R; #fields_count]>() };
                            #for_each_s             // Insert stream
                            res
                        }
                    
                        fn for_each_mut<F, R>(&mut self, mut func : F) -> [R; #fields_count]
                        where 
                            F : FnMut(&mut #comp_type, usize) -> R
                        {
                            let mut res = unsafe { core::mem::zeroed::<[R; #fields_count]>() };
                            #for_each_mut_s         // Insert stream
                            res
                        }
                    
                        fn try_for_each<'a, F, R, E>(&'a self, mut func : F) -> Result<[R; #fields_count], E>
                        where 
                            F : FnMut(&'a #comp_type, usize) -> Result<R, E>
                        {
                            let mut res = unsafe { core::mem::zeroed::<[R; #fields_count]>() };
                            #try_for_each_s         // Insert stream
                            Ok(res)
                        }
                    
                        fn try_for_each_mut<F, R, E>(&mut self, mut func : F) -> Result<[R; #fields_count], E>
                        where 
                            F : FnMut(&mut #comp_type, usize) -> Result<R, E>
                        {
                            let mut res = unsafe { core::mem::zeroed::<[R; #fields_count]>() };
                            #try_for_each_mut_s     // Insert stream
                            Ok(res)
                        } 
                    }
                }.into()
            },
            _ => panic!("This macro can only be used on structs")
        }
    }

    /// # `SyncActuatorGroup` - proc_macro
    /// 
    /// Automatically generates an implementation for the `SyncActuatorGroup` macro for a given struct if all the underlying compoments in the struct (all the fields)
    /// implment `SyncActuator` themselfs.
    #[proc_macro_derive(SyncActuatorGroup)]
    pub fn sync_comp_group_derive(input : proc_macro::TokenStream) -> proc_macro::TokenStream {
        let ast : DeriveInput = syn::parse(input).unwrap();
        sync_comp_group_impl(ast, TokenStream::from_str("(dyn syact::act::SyncActuator + 'static)").unwrap())
    }
// 

// StepperActuatorGroup
    /// Implementation for the `StepperActuatorGroup` trait
    fn stepper_comp_group_impl(ast : DeriveInput) -> proc_macro::TokenStream {
        match ast.data {
            syn::Data::Struct(data) => {
                // The macro only works on structs
                let name = ast.ident;
                let fields = data.fields;
                let fields_count = fields.len();

                // Create an empty implementation
                quote::quote! {
                    impl syact::act::stepper::StepperActuatorGroup<dyn syact::act::stepper::StepperActuator, #fields_count> for #name { }
                }.into()
            },
            _ => panic!("This macro can only be used on structs")
        }
    }

    /// # `StepperActuatorGroup` - proc_macro
    /// 
    /// Automatically generates an implementation for the `StepperActuatorGroup` macro for a given struct if all the underlying compoments in the struct (all the fields)
    /// implment `StepperActuator` themselfs.
    #[proc_macro_derive(StepperActuatorGroup)]
    pub fn stepper_comp_group_derive(input : proc_macro::TokenStream) -> proc_macro::TokenStream {
        let ast : DeriveInput = syn::parse(input).unwrap();
        let mut derive = sync_comp_group_impl(ast.clone(), TokenStream::from_str("(dyn syact::act::stepper::StepperActuator + 'static)").unwrap()); 
        derive.extend(stepper_comp_group_impl(ast));
        derive
    }
// 

