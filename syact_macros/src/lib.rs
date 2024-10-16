#![crate_name = "syact_macros"]
#![doc = include_str!("../README.md")]
#![deny(missing_docs)]

use core::str::FromStr;

use proc_macro2::TokenStream;
use syn::DeriveInput;

// SyncActuatorGroup
    fn actuator_group_implement(ast : DeriveInput, comp_type : TokenStream, item : TokenStream) -> proc_macro::TokenStream {
        match ast.data {
            // Macro works only on structs
            syn::Data::Struct(data) => {
                // Struct metadata
                let name = ast.ident;
                let fields = data.fields;
                let fields_count = fields.len();

                // Streams for different functions
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
                    #item

                    impl syact::act::group::ActuatorGroup<#comp_type, #fields_count> for #name { 
                        fn for_each<F, R>(&self, mut func : F) -> [R; #fields_count]
                        where 
                            F : FnMut(&#comp_type, usize) -> R
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
                    
                        fn try_for_each<F, R, E>(&self, mut func : F) -> Result<[R; #fields_count], E>
                        where 
                            F : FnMut(&#comp_type, usize) -> Result<R, E>
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
    /// Automatically generates an implementation for the `SyncActuatorGroup` macro for a given struct if all elements can be summarized as `dyn SyncActuator`
    #[proc_macro_derive(SyncActuatorGroup)]
    pub fn sync_actuator_group(item : proc_macro::TokenStream) -> proc_macro::TokenStream {
        let ast : DeriveInput = syn::parse(item).unwrap();
        actuator_group_implement(ast, TokenStream::from_str("(dyn syact::act::SyncActuator + 'static)").unwrap(), TokenStream::new())
    }

    /// # `StepperActuatorGroup` - proc_macro
    /// 
    /// Automatically generates an implementation for the `StepperActuatorGroup` macro for a given struct if all elements can be summarized as `dyn StepperActuator`
    #[proc_macro_derive(StepperActuatorGroup)]
    pub fn stepper_actuator_group(item : proc_macro::TokenStream) -> proc_macro::TokenStream {
        let ast : DeriveInput = syn::parse(item).unwrap();
        actuator_group_implement(ast, TokenStream::from_str("(dyn syact::act::StepperActuator + 'static)").unwrap(), TokenStream::new())
    }

    /// # `AdvancedActuatorGroup` - proc_macro
    /// 
    /// Automatically generates an implementation for the `StepperActuatorGroup` macro for a given struct if all elements can be summarized as `dyn AdvancedActuator`
    #[proc_macro_derive(AdvancedActuatorGroup)]
    pub fn advanced_actuator_group(item : proc_macro::TokenStream) -> proc_macro::TokenStream {
        let ast : DeriveInput = syn::parse(item).unwrap();
        actuator_group_implement(ast, TokenStream::from_str("(dyn syact::act::AdvancedActuator + 'static)").unwrap(), TokenStream::new())
    }

    /// # Free macro
    #[proc_macro_attribute]
    pub fn actuator_group(attr : proc_macro::TokenStream, item : proc_macro::TokenStream) -> proc_macro::TokenStream {
        let ast : DeriveInput = syn::parse(item.clone()).unwrap();
        actuator_group_implement(ast, attr.into(), item.into())
    }
// 